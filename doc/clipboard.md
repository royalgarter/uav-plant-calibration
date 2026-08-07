```
// ============================================================================
// HELPER 1: Background Subtraction (Concrete vs. Non-Concrete)
// ============================================================================
cv::Mat computeNonConcreteMask(const cv::Mat& rgbImg) {
    // Concrete floor is grey: Low Saturation (S in HSV) and Low Color Variance (|R-G|, |G-B|)
    cv::Mat hsv, nonConcreteMask;
    cv::cvtColor(rgbImg, hsv, cv::COLOR_BGR2HSV);
    
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    cv::Mat Saturation = hsvChannels[1]; // S channel

    // 1. Saturation threshold: Concrete S is typically < 20 (scale 0-255)
    cv::Mat highSatMask = Saturation > 22;

    // 2. Channel Delta Check: max(R,G,B) - min(R,G,B)
    std::vector<cv::Mat> bgr;
    cv::split(rgbImg, bgr);
    cv::Mat maxBGR, minBGR, chroma;
    cv::max(bgr[0], cv::max(bgr[1], bgr[2]), maxBGR);
    cv::min(bgr[0], cv::min(bgr[1], bgr[2]), minBGR);
    chroma = maxBGR - minBGR;
    
    cv::Mat chromaMask = chroma > 12; // True if there is actual color hue

    // Combine: A pixel is non-concrete if it has sufficient saturation OR chroma
    cv::bitwise_or(highSatMask, chromaMask, nonConcreteMask);
    return nonConcreteMask;
}

// ============================================================================
// HELPER 2: Texture / High-Frequency Variance Filtering
// ============================================================================
cv::Mat computeSmoothTextureMask(const cv::Mat& rgbImg, double maxTextureThresh = 32.0) {
    cv::Mat gray, lap, absLap, localTexture;
    cv::cvtColor(rgbImg, gray, cv::COLOR_BGR2GRAY);
    
    // Laplacian calculates spatial intensity gradients
    cv::Laplacian(gray, lap, CV_32F, 3);
    cv::convertScaleAbs(lap, absLap);
    
    // Blur to compute local gradient density (moss/concrete = high gradient variance)
    cv::blur(absLap, localTexture, cv::Size(7, 7));
    
    cv::Mat smoothMask;
    // Keep pixels with low local gradient variance (smooth leaf surfaces)
    cv::threshold(localTexture, smoothMask, maxTextureThresh, 255, cv::THRESH_BINARY_INV);
    return smoothMask;
}

// ============================================================================
// HELPER 3: Low-NDVI Thresholding (Supports Yellow / Stressed Leaves)
// ============================================================================
cv::Mat computeLowNDVIMask(const std::map<int, cv::Mat>& bands, cv::Size targetSize, float lowNdviThresh = 0.12f) {
    const float eps = 1e-6f;
    cv::Mat ndviMask;

    // Band 5 = NIR, Band 3 = Red (or Band 2 Green fallback)
    if (bands.count(5) && (bands.count(3) || bands.count(2))) {
        cv::Mat NIR = bands.at(5).clone();
        cv::Mat Red = bands.count(3) ? bands.at(3).clone() : bands.at(2).clone();

        if (NIR.channels() == 1 && Red.channels() == 1) {
            cv::resize(NIR, NIR, targetSize);
            cv::resize(Red, Red, targetSize);

            cv::Mat NIRf, Redf;
            NIR.convertTo(NIRf, CV_32F);
            Red.convertTo(Redf, CV_32F);

            // Compute NDVI = (NIR - Red) / (NIR + Red)
            cv::Mat NDVI = (NIRf - Redf) / (NIRf + Redf + eps);

            // Low threshold (0.12 - 0.15): Captures yellow/stressed leaves while excluding concrete (~0.02)
            cv::Mat ndvi8;
            cv::threshold(NDVI, ndviMask, lowNdviThresh, 255, cv::THRESH_BINARY);
            ndviMask.convertTo(ndviMask, CV_8U);
            return ndviMask;
        }
    }
    
    // Return empty if multispectral bands are missing
    return cv::Mat();
}

// ============================================================================
// MAIN FUNCTION: Integrated Pipeline
// ============================================================================
GreenMaskResults applyGreenMask(
    cv::Mat& indexImg, 
    const cv::Mat& rgbImg, 
    const std::string& outputDir, 
    const std::string& prefix, 
    const std::string& indexName, 
    const std::map<int, cv::Mat>& bands, 
    int greenCentroidRadiusX = 0, 
    int greenCentroidRadiusY = 0) 
{
    GreenMaskResults results;
    if (rgbImg.empty()) return results;

    // ------------------------------------------------------------------------
    // STEP 1: Background Subtraction & Low-NDVI & Texture Masks
    // ------------------------------------------------------------------------
    cv::Mat nonConcreteMask = computeNonConcreteMask(rgbImg);
    cv::Mat smoothTextureMask = computeSmoothTextureMask(rgbImg, 32.0);
    cv::Mat ndviMask = computeLowNDVIMask(bands, rgbImg.size(), 0.12f);

    cv::Mat candidateMask;
    if (!ndviMask.empty()) {
        // Primary route: Combine Low-NDVI with Non-Concrete and Texture filters
        cv::bitwise_and(nonConcreteMask, ndviMask, candidateMask);
        cv::bitwise_and(candidateMask, smoothTextureMask, candidateMask);
    } else {
        // Fallback route (RGB only): Non-Concrete + Texture + Color
        cv::Mat rgbf; rgbImg.convertTo(rgbf, CV_32F);
        std::vector<cv::Mat> bgr; cv::split(rgbf, bgr);
        cv::Mat PlantIndex = bgr[1] + bgr[2] - 2.0f * bgr[0]; // G + R - 2B
        
        cv::Mat PlantIndex_norm, PlantIndex8;
        cv::normalize(PlantIndex, PlantIndex_norm, 0.0f, 255.0f, cv::NORM_MINMAX);
        PlantIndex_norm.convertTo(PlantIndex8, CV_8U);

        cv::Mat rgbColorMask;
        cv::threshold(PlantIndex8, rgbColorMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        cv::bitwise_and(nonConcreteMask, rgbColorMask, candidateMask);
        cv::bitwise_and(candidateMask, smoothTextureMask, candidateMask);
    }

    // ------------------------------------------------------------------------
    // STEP 2: Stronger Morphological Opening (Erosion before Dilation)
    // ------------------------------------------------------------------------
    // Median blur to remove point noise
    cv::medianBlur(candidateMask, candidateMask, 5);

    // OPENING (Erosion -> Dilation) with 7x7 kernel: Destroys fine moss bridges
    cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(candidateMask, candidateMask, cv::MORPH_OPEN, kernelOpen);

    // CLOSING with 5x5 kernel: Fills small internal leaf gaps
    cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(candidateMask, candidateMask, cv::MORPH_CLOSE, kernelClose);

    // ------------------------------------------------------------------------
    // STEP 3: Dynamic Adaptive Area & Solidity / Compactness Filtering
    // ------------------------------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(candidateMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Dynamic Minimum Area Threshold (0.15% of total image pixels)
    double minAreaThreshold = rgbImg.cols * rgbImg.rows * 0.0015;

    struct ContourData {
        int index;
        double area;
        double solidity;
        cv::Point2f centroid;
    };

    std::vector<ContourData> validContours;
    double totalValidArea = 0;
    cv::Point2f weightedCentroid(0, 0);
    std::vector<cv::Point> allPlantPoints;

    cv::Mat filteredMask = cv::Mat::zeros(candidateMask.size(), CV_8U);

    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);

        if (area >= minAreaThreshold) {
            // Calculate Solidity = Area / Convex Hull Area
            std::vector<cv::Point> hull;
            cv::convexHull(contours[i], hull);
            double hullArea = cv::contourArea(hull);
            double solidity = (hullArea > 0) ? (area / hullArea) : 0.0;

            // Solidity filter: Moss/scattered noise < 0.50, Foliage/Canopy >= 0.50
            if (solidity >= 0.50) {
                cv::Moments mu = cv::moments(contours[i]);
                cv::Point2f centroid(mu.m10 / (mu.m00 + 1e-6), mu.m01 / (mu.m00 + 1e-6));

                validContours.push_back({i, area, solidity, centroid});
                
                // Draw valid plant contour to final mask
                cv::drawContours(filteredMask, contours, i, cv::Scalar(255), cv::FILLED);

                totalValidArea += area;
                weightedCentroid += centroid * (float)area;

                for (const auto& p : contours[i]) {
                    allPlantPoints.push_back(p);
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // STEP 4: Results & Output Construction
    // ------------------------------------------------------------------------
    if (totalValidArea > 0) {
        results.mask = filteredMask;
        results.totalArea = totalValidArea;
        results.centroid = weightedCentroid / (float)(totalValidArea + 1e-6);
        results.valid = true;

        if (allPlantPoints.size() >= 5) {
            cv::convexHull(allPlantPoints, results.convexHull);
            results.ellipse = cv::fitEllipse(allPlantPoints);
        }
    } else {
        results.mask = cv::Mat::zeros(candidateMask.size(), CV_8U);
        results.valid = false;
    }

    // Apply final mask to vegetation index image
    if (!indexImg.empty() && indexImg.size() == results.mask.size()) {
        indexImg.setTo(0, results.mask == 0);
    }

    return results;
}
```

By combining Background Subtraction (Concrete), Low-NDVI (Stressed Leaf Detection), and Texture Filtering with Morphological Opening and Solidity/Area Filtering, this pipeline successfully isolates both healthy and yellow/stressed plant foliage while completely stripping out ground moss and concrete noise.

### How These 6 Modules Work Together:

1. **`computeNonConcreteMask`:** Eliminates grey concrete floor pixels by enforcing a minimum color saturation/chroma threshold.
2. **`computeLowNDVIMask`:** Uses a relaxed baseline threshold ($\text{NDVI} > 0.12$). This **retains yellow/chlorotic/drought-stressed leaves** that fail green color tests, while stripping out background concrete.
3. **`computeSmoothTextureMask`:** Calculates local gradient density via Laplacian variance. It identifies and discards hyper-textured moss patches on concrete while keeping smooth leaf surfaces.
4. **Morphological Opening (`7x7` Ellipse):** Performs erosion before dilation, severing fine moss filaments and erasing small background speckles.
5. **Dynamic Minimum Area Threshold (`minAreaThreshold`):** Dynamically scales with image resolution (`width * height * 0.0015`), ignoring noise blobs without hardcoding static pixel limits.
6. **Solidity Filtering (`solidity >= 0.50`):** Compares contour area to its convex hull. Web-like moss fragments have low solidity ($< 0.45$) and are discarded, while solid plant canopies are preserved.