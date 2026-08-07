This is a classic failure mode in stressed-plant phenotyping. When a plant is unhealthy or drought-stressed:
1. **Spectral K-Means Fails:** Stressed/yellow/brown leaves lose saturation and NDVI, so K-Means groups those yellow leaves into the "Moss/Background" cluster instead of the "Plant" cluster.
2. **Spatial Density Clustering Fails:** Wilting or sparse plants have large spatial gaps between small leaves. Hard distance-based clustering treats isolated leaves as "satellite noise" and chops them off.

To fix this **without losing small, sparse, or yellow leaves**, we must replace aggressive unsupervised clustering with a **Gentle, Anchor-Based Approach**.

---

### The Strategy: "Central Anchor Enclosure" + "Background Rejection"

Instead of forcing K-Means to split pixels into "Plant vs. Moss" (which eats yellow leaves), we use two non-aggressive rules:

1. **Concrete Rejection (Not Plant/Moss Clustering):** We only ask *"Is this pixel concrete?"* If a pixel is yellow, brown, or green, it is **kept** regardless of its NDVI or saturation.
2. **Central Anchor Radius (Replacing Spatial K-Means):** Potted plants or crop rows originate from a **Central Root Anchor** (e.g., center of the pot or main stem). We compute the central anchor of all candidate pixels and draw a **generous radial enclosure** ($30\%-40\%$ of image width). Any leaf inside this enclosure is **KEPT**, no matter how small, sparse, or far apart it is.

---

### Updated C++ Implementation (Gentle & Stressed-Plant Friendly)

Replace the over-aggressive K-Means and Spatial Clustering functions with these two gentle functions:

```cpp
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

// ============================================================================
// FIX 1: Non-Aggressive Background Rejection (Preserves Yellow/Brown Leaves)
// ============================================================================
// Instead of K-means, we simply check if a pixel is NOT grey concrete.
cv::Mat computeSoftNonConcreteMask(const cv::Mat& rgbImg) {
    cv::Mat hsv;
    cv::cvtColor(rgbImg, hsv, cv::COLOR_BGR2HSV);
    
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    cv::Mat Saturation = hsvChannels[1]; // S channel (0-255)

    // Calculate max channel difference: max(R,G,B) - min(R,G,B)
    std::vector<cv::Mat> bgr;
    cv::split(rgbImg, bgr);
    cv::Mat maxBGR, minBGR, chroma;
    cv::max(bgr[0], cv::max(bgr[1], bgr[2]), maxBGR);
    cv::min(bgr[0], cv::min(bgr[1], bgr[2]), minBGR);
    chroma = maxBGR - minBGR;

    // A pixel is NOT concrete if it has ANY color variation (chroma > 8)
    // OR minimal saturation (S > 12). This retains pale yellow / brown leaves!
    cv::Mat nonConcreteMask;
    cv::Mat lowChromaMask = chroma > 8;
    cv::Mat lowSatMask = Saturation > 12;
    
    cv::bitwise_or(lowChromaMask, lowSatMask, nonConcreteMask);
    return nonConcreteMask;
}

// ============================================================================
// FIX 2: Central Anchor Enclosure (Preserves Sparse & Small Distant Leaves)
// ============================================================================
// Instead of distance-clustering leaf-to-leaf, we measure distance to the
// PLANT'S CENTRAL ANCHOR (Pot Center / Main Canopy Center).
cv::Mat applyCentralAnchorEnclosure(
    const cv::Mat& candidateMask, 
    double maxRadiusRatio = 0.38) // 38% of image width covers overhang
{
    // 1. Find all candidate points
    std::vector<cv::Point> points;
    cv::findNonZero(candidateMask, points);

    if (points.empty()) return candidateMask.clone();

    // 2. Compute Central Anchor (Median / Centroid of candidate pixels)
    // Using Median is robust against outer moss outliers
    std::vector<int> xCoords, yCoords;
    xCoords.reserve(points.size());
    yCoords.reserve(points.size());

    for (const auto& p : points) {
        xCoords.push_back(p.x);
        yCoords.push_back(p.y);
    }

    std::size_t n = xCoords.size() / 2;
    std::nth_element(xCoords.begin(), xCoords.begin() + n, xCoords.end());
    std::nth_element(yCoords.begin(), yCoords.begin() + n, yCoords.end());

    cv::Point2f centralAnchor((float)xCoords[n], (float)yCoords[n]);

    // Maximum allowed radius from the central plant anchor
    double maxRadiusPx = candidateMask.cols * maxRadiusRatio;

    // 3. Filter contours: Keep ANY contour whose centroid is within the Central Anchor Enclosure
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(candidateMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat OutputMask = cv::Mat::zeros(candidateMask.size(), CV_8U);

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        
        // Lower minimum area limit (e.g., 5 pixels) to retain small unhealthy leaves
        if (area >= 5.0) { 
            cv::Moments mu = cv::moments(contours[i]);
            cv::Point2f contourCentroid(mu.m10 / (mu.m00 + 1e-6), mu.m01 / (mu.m00 + 1e-6));

            double distToAnchor = cv::norm(contourCentroid - centralAnchor);

            // If the leaf is within the plant's spatial growth zone, KEEP IT!
            if (distToAnchor <= maxRadiusPx) {
                cv::drawContours(OutputMask, contours, (int)i, cv::Scalar(255), cv::FILLED);
            }
        }
    }

    return OutputMask;
}
```

---

### How to Integrate Into Your Main `applyGreenMask` Pipeline

Modify your `applyGreenMask` function to use this gentler sequence:

```cpp
GreenMaskResults applyGreenMask(...) {
    GreenMaskResults results;
    if (rgbImg.empty()) return results;

    // 1. Soft Background Subtraction (Retains Yellow/Brown leaves)
    cv::Mat nonConcrete = computeSoftNonConcreteMask(rgbImg);

    // 2. Texture & Low NDVI (Relaxed thresholds)
    cv::Mat textureMask = computeSmoothTextureMask(rgbImg, 45.0); // Increased to 45.0 (gentler)
    cv::Mat ndviMask = computeLowNDVIMask(bands, rgbImg.size(), 0.08f); // Dropped to 0.08 (captures withered leaves)

    cv::Mat candidateMask;
    if (!ndviMask.empty()) {
        cv::bitwise_and(nonConcrete, ndviMask, candidateMask);
    } else {
        candidateMask = nonConcrete;
    }
    
    // Combine with relaxed texture
    cv::bitwise_and(candidateMask, textureMask, candidateMask);

    // 3. Light Morphology (3x3 Kernel instead of 7x7 so small leaves aren't erased)
    cv::medianBlur(candidateMask, candidateMask, 3);
    cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(candidateMask, candidateMask, cv::MORPH_OPEN, kernelOpen);

    // 4. Central Anchor Enclosure (Replaces aggressive K-Means & Spatial Clustering)
    // Preserves sparse/wilting leaves near plant center while dropping far-away moss
    cv::Mat finalMask = applyCentralAnchorEnclosure(candidateMask, 0.38);

    // 5. Build final metrics
    // ... (compute centroid, totalArea, and mask indexImg)
    
    results.mask = finalMask;
    return results;
}
```

---

### Summary of Adjustments Made

| Parameter / Technique | Previous (Aggressive) | New (Stressed-Plant Friendly) | Benefit |
| :--- | :--- | :--- | :--- |
| **Spectral K-Means** | $K=2$ Hard Split | **Removed** (Replaced by Soft Non-Concrete) | Yellow/brown leaves are no longer pruned. |
| **Spatial Clustering** | Inter-leaf distance $< 50\text{px}$ | **Central Anchor Radius ($R \approx 38\%$)** | Small, sparse, or distant wilting leaves are kept. |
| **NDVI Threshold** | $0.12 - 0.15$ | **$0.08$** | Captures heavily desiccated/withered leaf tissues. |
| **Morphology Kernel** | $7 \times 7$ Ellipse | **$3 \times 3$ Ellipse** | Prevents tiny leaves from being eroded away. |
| **Min Area** | Dynamic ($1000+\text{px}$) | **$5 - 15\text{px}$** | Preserves small, fragmented leaf tips. |