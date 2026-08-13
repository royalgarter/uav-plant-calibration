#ifndef CALIB_H
#define CALIB_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <filesystem>
#include <opencv2/core.hpp>

// --- CONFIGURATION ---
struct ECCOptimization {
	bool enabled = true;
	int downscaleFactor = 2; // Reduce image size (2 means 1/2 width/height)
	int maxIterations = 50;  // Reduced from 500 for speed
	double epsilon = 1e-4;   // Relaxed from 1e-6
};

struct Config {
	ECCOptimization ecc;
};

// --- LOGGING UTILITY ---
struct Logger {
	std::ofstream file;
	bool fileOpen = false;
	std::mutex mtx;

	void open(const std::string& filename) {
		std::lock_guard<std::mutex> lock(mtx);
		file.open(filename);
		fileOpen = file.is_open();
	}

	template<typename T>
	Logger& operator<<(const T& msg) {
		std::lock_guard<std::mutex> lock(mtx);
		std::cout << msg;
		if (fileOpen) file << msg;
		return *this;
	}

	// For endl/iomanip
	Logger& operator<<(std::ostream& (*f)(std::ostream&)) {
		std::lock_guard<std::mutex> lock(mtx);
		f(std::cout);
		if (fileOpen) f(file);
		return *this;
	}
};

// --- IMAGE INFO ---
struct ImageInfo {
	std::string path;
	std::string filename;
	std::string uuid;
	std::string ext;

	// Distortion
	double fx = 0, fy = 0, cx_d = 0, cy_d = 0;
	double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
	bool foundDistortion = false;
	double calibratedCx = 0, calibratedCy = 0;
	uint32_t width = 0, height = 0;

	// Alignment
	double relX = 0, relY = 0;
	cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
	bool foundH = false;
};

// --- RADIOMETRIC CALIBRATION ---
struct RadioCoeffs {
	double a = 1.0;
	double b = 0.0;
	bool valid = false;
	// For RGB images (band 0), store per-channel coefficients
	double a_r = 1.0, b_r = 0.0;
	double a_g = 1.0, b_g = 0.0;
	double a_b = 1.0, b_b = 0.0;
	bool isRGB = false;

	// Data for CSV export
	std::string filename;
	std::string bandName;
	std::vector<double> targets; // For single-band images (e.g., NIR, Red, etc.)
	std::vector<double> dns;     // For single-band images
	std::vector<double> targets_r, targets_g, targets_b; // For RGB channels (Red, Green, Blue)
	std::vector<double> dns_r, dns_g, dns_b;     // For RGB channels

	// Anchor points for collecting DN values from all images
	cv::Point p56, p3;
	int boxSize = 5;
};

struct RadioRef {
	std::vector<double> patches; // 4 values: patch56, patch36, patch12, patch3
};

struct GroupData {
	std::string prefix;
	std::vector<ImageInfo> images;
	ImageInfo* refInfo = nullptr;
	RadioCoeffs coeffs;
	// Store DN values from all images for CSV export
	// multispectralDns[bands 1-5][4 anchor points]
	std::map<int, std::vector<double>> multispectralDns;
	// rgbDns[3 channels: R,G,B][4 anchor points]
	std::vector<double> rgbDns_r, rgbDns_g, rgbDns_b;
	bool rgbCollected = false;
};

struct RadioState {
	cv::Point p56;
	cv::Point p3;
	int clicks = 0;
};

// --- GLOBALS (extern) ---
extern Logger gLog;
extern Config gConfig;
extern std::map<std::string, RadioRef> gRadioRefs;

// --- FUNCTION PROTOTYPES ---

// Metadata
void parseXmlMetadata(const std::string& xml, ImageInfo& info);
std::string getXmpFromJpeg(const std::string& filename);
ImageInfo parseMetadata(const std::string& filePath);

// Image Processing
cv::Mat undistortImg(const cv::Mat& img, const ImageInfo& info);
cv::Mat getCLAHE(const cv::Mat& img);
cv::Mat prepareForECC(const cv::Mat& img);
cv::Mat alignSIFTFallback(const cv::Mat& refClahe, const cv::Mat& targetClahe);
cv::Mat contrastStretch(const cv::Mat& src);

// Radiometric
bool loadRadiometricRefs(const std::string& path);
void collectDnValues(const cv::Mat& img, const cv::Point& p56, const cv::Point& p3, int boxSize,
                     bool isRGB, std::vector<double>& dns_r, std::vector<double>& dns_g, std::vector<double>& dns_b);
RadioCoeffs getRadiometricCoeffs(const cv::Mat& img, const std::string& filename, cv::Point interval, int autoDetectThickness = -1, 
                                  const std::string& radioDir = ".output/radio", const std::string& templatePath = ".input_ref/radiometric_board.jpg");
cv::Mat applyRadiometricCalibration(const cv::Mat& img, RadioCoeffs coeffs);
void exportRadiometricCsv(const std::string& outPath, const std::map<std::string, GroupData>& allGroups);

// Preprocessing (green mask noise filtering) parameters
struct GreenMaskParams {
	int centroidCenterX = -1;       // ROI ellipse center X (px); -1 = image center
	int centroidCenterY = -1;       // ROI ellipse center Y (px); -1 = image center
	int centroidRadiusX = 0;        // ROI ellipse radius X (px); 0=disabled
	int centroidRadiusY = 0;        // ROI ellipse radius Y (px); 0=disabled
	
	double textureThresh = 32.0;    // smooth-texture max gradient threshold
	double ndviThresh = 0.12;       // low-NDVI baseline threshold
	double minAreaRatio = 0.0015;   // min contour area as fraction of image pixels
	double solidityThresh = 0.50;   // min contour solidity to keep
	
	int medianBlurSize = 5;         // median blur kernel size
	int openKernelSize = 7;         // morphological opening kernel size
	int closeKernelSize = 5;        // morphological closing kernel size
	
	bool spectralKMeans = false;    // refine big blobs via [H,S,NDVI] K-Means (K=2)
	bool spatialCluster = false;    // drop satellite moss blobs via spatial proximity grouping
	int spatialMergeDist = 50;      // max centroid distance (px) to merge contours into one cluster

	// Gentle (stressed-plant) mode: preserves yellow/brown/sparse/small leaves
	bool gentleMode = false;        // use soft background rejection + central anchor enclosure
	
	double anchorRadiusRatio = 0.38; // central anchor radial enclosure as fraction of image width
	double gentleTextThresh = 45.0;  // relaxed texture threshold (smoother = kept)
	double gentleNdviThresh = 0.08;  // relaxed low-NDVI threshold (captures withered leaves)
	
	int gentleOpenKernel = 3;        // light morphological kernel so tiny leaves survive
	int gentleMinArea = 5;           // min contour area (px) to retain small leaf tips

	// Color-spot HSV leaf selection: replaces the default green color filter.
	struct ColorSpot { int h = 0; int s = 0; int v = 0; }; // HSV center (H 0-180)
	std::vector<ColorSpot> colorSpots; // empty => default green filter preset
	int hsvTolH = 15;                  // global hue tolerance (deg/2, H 0-180)
	int hsvTolS = 40;                  // global saturation tolerance
	int hsvTolV = 40;                  // global value tolerance
};

// Vegetation Indices
struct GreenMaskResults {
	cv::Mat mask;
	cv::Point2f centroid = cv::Point2f(0, 0);
	std::vector<cv::Point> convexHull;
	cv::RotatedRect ellipse;
	double totalArea = 0;
	bool valid = false;
};

cv::Mat refineMaskWithKMeansRegion(const cv::Mat& candidateMask, const cv::Mat& rgbImg, const cv::Mat& ndviImg);
cv::Mat filterSatelliteClusters(const cv::Mat& inputMask, double maxMergeDistancePx = 50.0);
cv::Mat computeColorSpotMask(const cv::Mat& rgbImg, const GreenMaskParams& params);
cv::Mat computeSoftNonConcreteMask(const cv::Mat& rgbImg);
cv::Mat applyCentralAnchorEnclosure(const cv::Mat& candidateMask, double maxRadiusRatio = 0.38);
cv::Mat calculateVegIndex(const std::string& type, const std::map<int, cv::Mat>& bands, const cv::Mat& mask = cv::Mat());
GreenMaskResults applyGreenMask(cv::Mat& indexImg, const cv::Mat& rgbImg, const std::string& outputDir, const std::string& prefix, const std::string& indexName, const std::map<int, cv::Mat>& bands, const GreenMaskParams& params = GreenMaskParams());
void exportVegIndexCsv(const std::string& outPath, const std::vector<std::string>& requestedIndices, const std::map<std::string, std::map<std::string, double>>& averages);

#endif // CALIB_H
