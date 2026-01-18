#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <vector>

// Check if we can include libtiff
#if __has_include(<tiffio.h>)
    #include <tiffio.h>
    #define HAS_LIBTIFF 1
#else
    #define HAS_LIBTIFF 0
#endif

int main() {
    std::cout << "Checking TIFF support..." << std::endl;
    std::string filename = "test_gen.tif";

    // 1. Generate a TIFF using OpenCV
    cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 255, 0)); // Green square
    bool saved = cv::imwrite(filename, image);
    if (saved) {
        std::cout << "[SUCCESS] OpenCV wrote " << filename << std::endl;
    } else {
        std::cerr << "[FAILURE] OpenCV failed to write " << filename << std::endl;
        return 1;
    }

    // 2. Read it back using OpenCV
    cv::Mat readImg = cv::imread(filename, cv::IMREAD_UNCHANGED);
    if (!readImg.empty()) {
        std::cout << "[SUCCESS] OpenCV read " << filename << std::endl;
        std::cout << "Dimensions: " << readImg.cols << "x" << readImg.rows << std::endl;
    } else {
        std::cerr << "[FAILURE] OpenCV failed to read " << filename << std::endl;
    }

    // 3. Check for Metadata support via libtiff
    if (HAS_LIBTIFF) {
        std::cout << "[INFO] libtiff headers found. Checking metadata access..." << std::endl;
        TIFF* tif = TIFFOpen(filename.c_str(), "r");
        if (tif) {
            uint32_t width, height;
            TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
            TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
            std::cout << "[SUCCESS] libtiff read metadata: " << width << "x" << height << std::endl;
            TIFFClose(tif);
        } else {
            std::cerr << "[FAILURE] libtiff failed to open " << filename << std::endl;
        }
    } else {
        std::cout << "[INFO] libtiff headers NOT found. Cannot verify direct metadata access." << std::endl;
    }

    return 0;
}
