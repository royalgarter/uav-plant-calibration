#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <string>
#include <algorithm>
#include <cctype>

namespace fs = std::filesystem;

std::vector<cv::Point3f> calibratePattern(cv::Size checkboardSize, float squareSize) {
    std::vector<cv::Point3f> ret;
    for (int i = 0; i < checkboardSize.height; i++) {
        for (int j = 0; j < checkboardSize.width; j++) {
            ret.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
        }
    }
    return ret;
}

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cout << "Usage: ./fisheye <src_image> <dest_image> <samples_dir> <checkboard_width> <checkboard_height>" << std::endl;
        return 1;
    }

    std::string srcPath = argv[1];
    std::string destPath = argv[2];
    std::string samplesDir = argv[3];
    int checkboardWidth = std::stoi(argv[4]);
    int checkboardHeight = std::stoi(argv[5]);

    // 1. Load Calibration Images
    std::vector<cv::Mat> images;
    std::cout << "Loading samples from " << samplesDir << "..." << std::endl;
    
    if (!fs::exists(samplesDir)) {
        std::cerr << "Error: Samples directory '" << samplesDir << "' not found." << std::endl;
        return 1;
    }

    for (const auto& entry : fs::directory_iterator(samplesDir)) {
        std::string p = entry.path().string();
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c){ return std::tolower(c); });

        // Simple check for jpg/png
        if (ext == ".jpg" || ext == ".png") {
            cv::Mat img = cv::imread(p, cv::IMREAD_GRAYSCALE);
            if (!img.empty()) {
                images.push_back(img);
            }
        }
    }

    if (images.empty()) {
        std::cerr << "No images found in " << samplesDir << std::endl;
        return 1;
    }
    std::cout << "Loaded " << images.size() << " images." << std::endl;

    // 2. Calibrate
    std::cout << "Calibrating..." << std::endl;
    cv::Size checkboardSize(checkboardWidth, checkboardHeight);
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<cv::Mat> imgPoints;
    std::vector<cv::Point3f> pattern = calibratePattern(checkboardSize, 1.0);
    cv::TermCriteria subpixCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1);

    for (auto& img : images) {
        cv::Mat corners;
        bool found = cv::findChessboardCorners(img, checkboardSize, corners);
        if (found) {
            objPoints.push_back(pattern);
            cv::cornerSubPix(img, corners, cv::Size(3, 3), cv::Size(-1, -1), subpixCriteria);
            imgPoints.push_back(corners);
        }
    }

    cv::Matx33d K;
    cv::Vec4d D;
    cv::Size size = images[0].size();
    int flag = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND | cv::fisheye::CALIB_FIX_SKEW;
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 1e-6);
    
    double error = cv::fisheye::calibrate(objPoints, imgPoints, size, K, D, cv::noArray(), cv::noArray(), flag, criteria);
    std::cout << "Calibration done. Reprojection error: " << error << std::endl;

    // 3. Undistort
    std::cout << "Undistorting " << srcPath << "..." << std::endl;
    cv::Mat distorted = cv::imread(srcPath);
    if (distorted.empty()) {
        std::cerr << "Failed to read source image: " << srcPath << std::endl;
        return 1;
    }

    cv::Mat undistorted;
    // Note: The example JS implementation uses K for both camera matrix and new camera matrix, 
    // effectively preserving the scale/view but removing distortion.
    cv::fisheye::undistortImage(distorted, undistorted, K, D, K, distorted.size());

    // 4. Save
    if (cv::imwrite(destPath, undistorted)) {
        std::cout << "Saved to " << destPath << std::endl;
    } else {
        std::cerr << "Failed to save image to " << destPath << std::endl;
        return 1;
    }

    return 0;
}
