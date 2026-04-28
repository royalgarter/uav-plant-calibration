#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <regex>
#include <sstream>
#include <vector>
#include <map>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <omp.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp> // For findTransformECC
#include <opencv2/features2d.hpp> // For SIFT
#include <opencv2/highgui.hpp>
#include <tiffio.h>

// Uncomment to enable Windows GUI support
// #define WINGUI

#ifdef WINGUI
#include "calib-raygui.cc"
#endif

using namespace std;
using namespace std::filesystem;
using namespace cv;

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

Config gConfig;

// --- LOGGING UTILITY ---
struct Logger {
	ofstream file;
	bool fileOpen = false;
	mutex mtx;

	void open(const string& filename) {
		lock_guard<mutex> lock(mtx);
		file.open(filename);
		fileOpen = file.is_open();
	}

	template<typename T>
	Logger& operator<<(const T& msg) {
		lock_guard<mutex> lock(mtx);
		cout << msg;
		if (fileOpen) file << msg;
		return *this;
	}

	// For endl/iomanip
	Logger& operator<<(ostream& (*f)(ostream&)) {
		lock_guard<mutex> lock(mtx);
		f(cout);
		if (fileOpen) f(file);
		return *this;
	}
};

Logger gLog;

struct ImageInfo {
	string path;
	string filename;
	string uuid;
	string ext;

	// Distortion
	double fx = 0, fy = 0, cx_d = 0, cy_d = 0;
	double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
	bool foundDistortion = false;
	double calibratedCx = 0, calibratedCy = 0;
	uint32_t width = 0, height = 0;

	// Alignment
	double relX = 0, relY = 0;
	Mat H = Mat::eye(3, 3, CV_64F);
	bool foundH = false;
};

// Helper to parse the XML metadata string
void parseXmlMetadata(const string& xml, ImageInfo& info) {
	smatch m;

	// CaptureUUID
	if (regex_search(xml, m, regex(R"(drone-dji:CaptureUUID=([^"]+))") ))
		info.uuid = m[1];

	// 1. Parse Calibrated Optical Center
	if (regex_search(xml, m, regex(R"(drone-dji:CalibratedOpticalCenterX=\"([^"]+)\")") ))
		info.calibratedCx = stod(m[1]);
	if (regex_search(xml, m, regex(R"(drone-dji:CalibratedOpticalCenterY=\"([^"]+)\")") ))
		info.calibratedCy = stod(m[1]);

	// 2. Parse Relative Optical Center
	if (regex_search(xml, m, regex(R"(drone-dji:RelativeOpticalCenterX=\"([^"]+)\")") ))
		info.relX = stod(m[1]);
	if (regex_search(xml, m, regex(R"(drone-dji:RelativeOpticalCenterY=\"([^"]+)\")") ))
		info.relY = stod(m[1]);

	cout << info.filename << ", " << info.calibratedCx << ", " << info.calibratedCy << ", " << info.relX << ", " << info.relY << '\n';

	// 3. Parse DewarpData
	if (regex_search(xml, m, regex(R"(drone-dji:DewarpData=\"([^"]+)\")") )) {
		string dataStr = m[1];
		size_t semiPos = dataStr.find(';');

		cout << dataStr << '\n';

		if (semiPos != string::npos) {
			string paramsStr = dataStr.substr(semiPos + 1);
			stringstream ss(paramsStr);
			string segment;
			vector<double> v;
			while(getline(ss, segment, ',')) v.push_back(stod(segment));

			if (v.size() >= 9) {
				info.fx = v[0]; info.fy = v[1];
				info.cx_d = v[2]; info.cy_d = v[3];
				info.k1 = v[4]; info.k2 = v[5]; info.p1 = v[6]; info.p2 = v[7]; info.k3 = v[8];
				info.foundDistortion = true;
			}
		}
	}

	// 4. DJI Matrix
	if (regex_search(xml, m, regex(R"(drone-dji:DewarpHMatrix=\"([^"]+)\")") )) {
		string matrixStr = m[1];
		cout << matrixStr << '\n';
		stringstream ss(matrixStr);
		string segment;
		vector<double> values;
		while(getline(ss, segment, ',')) {
			values.push_back(stod(segment));
		}
		if (values.size() == 9) {
			for(int i=0; i<3; i++) {
				for(int j=0; j<3; j++) {
					info.H.at<double>(i, j) = values[i*3 + j];
				}
			}
			info.foundH = true;
		}
	}
}

string getXmpFromJpeg(const string& filename) {
    FILE* f = fopen(filename.c_str(), "rb");
    if (!f) return "";

    uint8_t buf[256];
    // Read SOI
    if (fread(buf, 1, 2, f) != 2 || buf[0] != 0xFF || buf[1] != 0xD8) {
        fclose(f);
        return "";
    }

    while (true) {
        if (fread(buf, 1, 2, f) != 2) break; // Read marker
        if (buf[0] != 0xFF) break; // Not a marker
        uint8_t marker = buf[1];

        // Read length
        uint8_t lenBuf[2];
        if (fread(lenBuf, 1, 2, f) != 2) break;
        uint16_t len = (lenBuf[0] << 8) | lenBuf[1];
        uint16_t contentLen = len - 2;

        if (marker == 0xE1) { // APP1
             // Check for XMP header
             // "http://ns.adobe.com/xap/1.0/\0" is 29 bytes
             if (contentLen > 29) {
                 char header[29];
                 if (fread(header, 1, 29, f) != 29) break;
                 if (memcmp(header, "http://ns.adobe.com/xap/1.0/", 29) == 0) {
                     // Found XMP
                     string xmp;
                     xmp.resize(contentLen - 29);
                     if (fread(&xmp[0], 1, contentLen - 29, f) != contentLen - 29) break;
                     fclose(f);
                     return xmp;
                 } else {
                     // Not XMP, skip rest of segment
                     fseek(f, contentLen - 29, SEEK_CUR);
                 }
             } else {
                 fseek(f, contentLen, SEEK_CUR);
             }
        } else if (marker == 0xD9 || marker == 0xDA) {
            // EOI or SOS - stop scanning
            break;
        } else {
            // Skip other segments
            fseek(f, contentLen, SEEK_CUR);
        }
    }
    fclose(f);
    return "";
}

ImageInfo parseMetadata(const string& filePath) {
	ImageInfo info;
	info.path = filePath;
	info.filename = path(filePath).filename().string();
	info.ext = info.filename.substr(info.filename.find_last_of(".") + 1);

	// Check for TIFF extension before trying TIFFOpen to avoid warnings/errors on JPEGs
	bool isTiff = (info.ext == "tif" || info.ext == "TIF" || info.ext == "tiff" || info.ext == "TIFF");

	if (isTiff) {
		TIFF* tif = TIFFOpen(filePath.c_str(), "r");
		if (tif) {
			TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &info.width);
			TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &info.height);

			void* data;
			uint32_t len;
			if (TIFFGetField(tif, 700, &len, &data)) {
				string xml((char*)data, len);
				parseXmlMetadata(xml, info);
			}
			TIFFClose(tif);
			return info;
		}
	}

	// Fallback for non-TIFF files (like JPG) or if TIFF parsing failed
	// 1. Try to read XMP from JPEG structure
	string xmp = getXmpFromJpeg(filePath);
	if (!xmp.empty()) {
		parseXmlMetadata(xmp, info);
	}

	// 2. Read dimensions via OpenCV (robust fallback)
	Mat img = imread(filePath, IMREAD_UNCHANGED);
	if (!img.empty()) {
		info.width = img.cols;
		info.height = img.rows;
	}

	return info;
}

Mat undistortImg(const Mat& img, const ImageInfo& info) {
	if (info.foundDistortion) {
		double centerX = info.width > 0 ? info.width / 2.0 : info.calibratedCx;
		double centerY = info.height > 0 ? info.height / 2.0 : info.calibratedCy;

		// Matches drnmppr-dewarp.cpp logic:
		// cx = Width/2 - dewarp_cx
		// cy = Height/2 + dewarp_cy
		double finalCx = centerX - info.cx_d;
		double finalCy = centerY + info.cy_d;

		Mat K = (Mat_<double>(3, 3) << info.fx, 0, finalCx, 0, info.fy, finalCy, 0, 0, 1);
		Mat D = (Mat_<double>(1, 5) << info.k1, info.k2, info.p1, info.p2, info.k3);

		Mat dewarped;
		undistort(img, dewarped, K, D, K);
		return dewarped;
	}
	return img.clone();
}

Mat getCLAHE(const Mat& img) {
	Mat gray;
	if (img.channels() > 1) cvtColor(img, gray, COLOR_BGR2GRAY);
	else gray = img.clone();

	Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
	Mat img_clahe;
	clahe->apply(gray, img_clahe);
	return img_clahe;
}

Mat prepareForECC(const Mat& img) {
	Mat gray = getCLAHE(img);

	// 2. Gaussian Blur to reduce high-frequency sensor noise
	Mat blurred;
	GaussianBlur(gray, blurred, Size(5, 5), 0);

	// 3. Calculate X and Y gradients using Scharr (better rotational symmetry than Sobel)
	Mat grad_x, grad_y;
	Scharr(blurred, grad_x, CV_32F, 1, 0);
	Scharr(blurred, grad_y, CV_32F, 0, 1);

	// 4. Calculate Gradient Magnitude (removes polarity, keeps structure)
	Mat magnitude;
	cv::magnitude(grad_x, grad_y, magnitude);

	// 5. Normalize for numerical stability in ECC
	normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);

	return magnitude;
}

Mat alignSIFTFallback(const Mat& refClahe, const Mat& targetClahe) {
	Ptr<SIFT> sift = SIFT::create();
	vector<KeyPoint> kp1, kp2;
	Mat des1, des2;

	sift->detectAndCompute(refClahe, noArray(), kp1, des1);
	sift->detectAndCompute(targetClahe, noArray(), kp2, des2);

	if (des1.empty() || des2.empty()) return Mat();

	BFMatcher matcher(NORM_L2);
	vector<vector<DMatch>> knn_matches;
	matcher.knnMatch(des1, des2, knn_matches, 2);

	vector<DMatch> good_matches;
	for (size_t i = 0; i < knn_matches.size(); i++) {
		if (knn_matches[i][0].distance < 0.75 * knn_matches[i][1].distance) {
			good_matches.push_back(knn_matches[i][0]);
		}
	}

	if (good_matches.size() > 10) {
		vector<Point2f> src_pts, dst_pts;
		for (size_t i = 0; i < good_matches.size(); i++) {
			src_pts.push_back(kp1[good_matches[i].queryIdx].pt);
			dst_pts.push_back(kp2[good_matches[i].trainIdx].pt);
		}

		// Estimate Homography using RANSAC
		// We use findHomography to be consistent with the ECC MOTION_HOMOGRAPHY used in the main loop
		Mat H = findHomography(dst_pts, src_pts, RANSAC);
		return H;
	}

	return Mat();
}

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
	string filename;
	string bandName;
	vector<double> targets; // For single-band images (e.g., NIR, Red, etc.)
	vector<double> dns;     // For single-band images
	vector<double> targets_r, targets_g, targets_b; // For RGB channels (Red, Green, Blue)
	vector<double> dns_r, dns_g, dns_b;     // For RGB channels

	// Anchor points for collecting DN values from all images
	Point p56, p3;
	int boxSize = 5;
};

// Radiometric reference data loaded from config file
struct RadioRef {
	vector<double> patches; // 4 values: patch56, patch36, patch12, patch3
};
map<string, RadioRef> gRadioRefs; // Key: band identifier ("5", "4", "3", "2", "1", "0R", "0G", "0B")

// Load radiometric reference data from CSV file
bool loadRadiometricRefs(const string& path) {
	ifstream file(path);
	if (!file.is_open()) {
		gLog << "  WARNING: Could not load radiometric reference file: " << path << endl;
		return false;
	}

	string line;
	while (getline(file, line)) {
		// Skip empty lines and comments
		if (line.empty() || line[0] == '#') continue;
		// Skip header line
		if (line.find("band,") == 0) continue;

		stringstream ss(line);
		string band, p56, p36, p12, p3;
		getline(ss, band, ',');
		getline(ss, p56, ',');
		getline(ss, p36, ',');
		getline(ss, p12, ',');
		getline(ss, p3, ',');

		RadioRef ref;
		ref.patches = {
			stod(p56),
			stod(p36),
			stod(p12),
			stod(p3)
		};
		gRadioRefs[band] = ref;
	}

	file.close();
	gLog << "  Loaded " << gRadioRefs.size() << " radiometric reference entries from: " << path << endl;
	return true;
}

struct GroupData {
	string prefix;
	vector<ImageInfo> images;
	ImageInfo* refInfo = nullptr;
	RadioCoeffs coeffs;
	// Store DN values from all images for CSV export
	// multispectralDns[bands 1-5][4 anchor points]
	map<int, vector<double>> multispectralDns;
	// rgbDns[3 channels: R,G,B][4 anchor points]
	vector<double> rgbDns_r, rgbDns_g, rgbDns_b;
	bool rgbCollected = false;
};

struct RadioState {
	Point p56;
	Point p3;
	int clicks = 0;
};

void onMouseRadio(int event, int x, int y, int flags, void* userdata) {
	if (event == EVENT_LBUTTONDOWN) {
		RadioState* state = (RadioState*)userdata;
		if (state->clicks == 0) {
			state->p56 = Point(x, y);
			state->clicks = 1;
		} else if (state->clicks == 1) {
			state->p3 = Point(x, y);
			state->clicks = 2;
		}
	}
}

// Helper function to collect DN values from an image using given anchor points
void collectDnValues(const Mat& img, const Point& p56, const Point& p3, int boxSize,
                     bool isRGB, vector<double>& dns_r, vector<double>& dns_g, vector<double>& dns_b) {
	dns_r.clear();
	dns_g.clear();
	dns_b.clear();

	double stepX = (p3.x - p56.x) / 3.0;
	double stepY = (p3.y - p56.y) / 3.0;

	for (int i = 0; i < 4; ++i) {
		int cx = cvRound(p56.x + i * stepX);
		int cy = cvRound(p56.y + i * stepY);

		Rect roi(cx - boxSize / 2, cy - boxSize / 2, boxSize, boxSize);
		roi &= Rect(0, 0, img.cols, img.rows);

		if (roi.area() > 0) {
			Scalar avg = mean(img(roi));
			if (isRGB && img.channels() >= 3) {
				// For RGB images, collect per-channel values (OpenCV uses BGR order)
				dns_b.push_back(avg[0]);
				dns_g.push_back(avg[1]);
				dns_r.push_back(avg[2]);
			} else {
				// For multispectral single-band images
				dns_r.push_back(avg[0]);
			}
		} else {
			dns_r.push_back(0);
			if (isRGB) {
				dns_g.push_back(0);
				dns_b.push_back(0);
			}
		}
	}
}

RadioCoeffs getRadiometricCoeffs(const Mat& img, const string& filename, Point interval, int autoDetectThickness = -1, 
                                  const string& radioDir = ".output/radio", const string& templatePath = ".input_ref/radiometric_board.jpg") {
	RadioState state;
	Mat display;
	RadioCoeffs coeffs;

	// For display, we need 8-bit. We'll use a local normalization for viewing.
	if (img.depth() != CV_8U) {
		double minVal, maxVal;
		minMaxLoc(img, &minVal, &maxVal);
		if (maxVal == minVal) maxVal = minVal + 1.0;
		img.convertTo(display, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
	} else {
		display = img.clone();
	}

	if (display.channels() == 1) cvtColor(display, display, COLOR_GRAY2BGR);

	if (autoDetectThickness >= 0) {
		Mat templ = imread(templatePath, IMREAD_COLOR);
		if (!templ.empty()) {
			cout << "  Auto-detecting board using template..." << endl;
			double maxVal_total = -1;
			Point maxLoc_total;
			int bestRot = 0;
			double bestScale = 1.0;
			Size bestSize;

			Mat grayDisplay;
			cvtColor(display, grayDisplay, COLOR_BGR2GRAY);

			// Try different scales and rotations
			vector<double> scales = {0.5, 0.75, 1.0, 1.25, 1.5};
			for (double scale : scales) {
				Mat scaledTempl;
				resize(templ, scaledTempl, Size(), scale, scale);
				
				for (int rot = 0; rot < 4; ++rot) {
					Mat rotTempl;
					if (rot == 0) rotTempl = scaledTempl;
					else if (rot == 1) rotate(scaledTempl, rotTempl, ROTATE_90_CLOCKWISE);
					else if (rot == 2) rotate(scaledTempl, rotTempl, ROTATE_180);
					else if (rot == 3) rotate(scaledTempl, rotTempl, ROTATE_90_COUNTERCLOCKWISE);

					if (rotTempl.cols > grayDisplay.cols || rotTempl.rows > grayDisplay.rows) continue;

					Mat grayTempl;
					cvtColor(rotTempl, grayTempl, COLOR_BGR2GRAY);

					Mat result;
					matchTemplate(grayDisplay, grayTempl, result, TM_CCOEFF_NORMED);
					double minVal, maxVal;
					Point minLoc, maxLoc;
					minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

					if (maxVal > maxVal_total) {
						maxVal_total = maxVal;
						maxLoc_total = maxLoc;
						bestRot = rot;
						bestScale = scale;
						bestSize = rotTempl.size();
					}
				}
			}

			if (maxVal_total > 0.7) {
				cout << "    Board detected! (score=" << maxVal_total << ", rot=" << bestRot*90 << "deg, scale=" << bestScale << ")" << endl;
				
				// Calculate patch centers (56% and 3%)
				// In the vertical template, 56% is top, 3% is bottom.
				// We need to account for rotation and border thickness.
				Point p56_rel, p3_rel;
				
				double T = autoDetectThickness; // Use thickness directly as pixels
				double IW = bestSize.width - 2 * T;
				double IH = bestSize.height - 2 * T;

				if (bestRot == 0) { // Vertical: 56% top, 3% bottom
					p56_rel = Point(bestSize.width / 2, T + IH / 8);
					p3_rel = Point(bestSize.width / 2, T + IH * 7 / 8);
				} else if (bestRot == 1) { // 90 deg: 56% right, 3% left
					p56_rel = Point(T + IW * 7 / 8, bestSize.height / 2);
					p3_rel = Point(T + IW / 8, bestSize.height / 2);
				} else if (bestRot == 2) { // 180 deg: 56% bottom, 3% top
					p56_rel = Point(bestSize.width / 2, T + IH * 7 / 8);
					p3_rel = Point(bestSize.width / 2, T + IH / 8);
				} else if (bestRot == 3) { // 270 deg: 56% left, 3% right
					p56_rel = Point(T + IW / 8, bestSize.height / 2);
					p3_rel = Point(T + IW * 7 / 8, bestSize.height / 2);
				}

				state.p56 = maxLoc_total + p56_rel;
				state.p3 = maxLoc_total + p3_rel;
				state.clicks = 2;
			} else {
				cout << "    Board NOT detected (best score=" << maxVal_total << "). Falling back to manual." << endl;
			}
		} else {
			cout << "    Warning: Could not load template " << templatePath << ". Falling back to manual." << endl;
		}
	}

	string winName = "Radiometric Calibration - " + filename;
	if (state.clicks < 2) {
		namedWindow(winName, WINDOW_NORMAL);
		setMouseCallback(winName, onMouseRadio, &state);

		cout << "Radiometric Calibration for " << filename << ":" << endl;
		cout << "  1. Click on the center of the 56% patch (usually the brightest/leftmost)." << endl;
		cout << "  2. Click on the center of the 3% patch (usually the darkest/rightmost)." << endl;
		cout << "  Intermediate patches (36%, 12%) will be interpolated automatically." << endl;
		cout << "  Press ESC to skip calibration for this group." << endl;
	}

	int boxSize = 5;
	while (state.clicks < 2) {
		Mat temp = display.clone();
		if (state.clicks >= 1) {
			circle(temp, state.p56, boxSize, Scalar(0, 0, 255), -1);
		}
		imshow(winName, temp);
		int key = waitKey(10);
		if (key == 27) { // ESC
			destroyWindow(winName);
			cout << "  Skipped." << endl;
			return coeffs;
		}
	}

	// Store filename in coeffs
	coeffs.filename = filename;

	// Select targets based on spectral band (last character of filename stem)
	string stem = path(filename).stem().string();
	
	// Default targets if config not loaded
	vector<double> defaultTargets = {0.5647, 0.3582, 0.1148, 0.0272};
	vector<double> targets = defaultTargets;

	if (!stem.empty()) {
		char lastChar = stem.back();
		
		if (lastChar == '5') {
			auto it = gRadioRefs.find("5");
			if (it != gRadioRefs.end()) targets = it->second.patches;
			else targets = defaultTargets;
			coeffs.bandName = "NIR";
		}
		else if (lastChar == '4') {
			auto it = gRadioRefs.find("4");
			if (it != gRadioRefs.end()) targets = it->second.patches;
			else targets = defaultTargets;
			coeffs.bandName = "RedEdge";
		}
		else if (lastChar == '3') {
			auto it = gRadioRefs.find("3");
			if (it != gRadioRefs.end()) targets = it->second.patches;
			else targets = defaultTargets;
			coeffs.bandName = "Red";
		}
		else if (lastChar == '2') {
			auto it = gRadioRefs.find("2");
			if (it != gRadioRefs.end()) targets = it->second.patches;
			else targets = defaultTargets;
			coeffs.bandName = "Green";
		}
		else if (lastChar == '1') {
			auto it = gRadioRefs.find("1");
			if (it != gRadioRefs.end()) targets = it->second.patches;
			else targets = defaultTargets;
			coeffs.bandName = "Blue";
		}
		else if (lastChar == '0') {
			// For RGB image (No.0), calibrate each channel separately
			coeffs.isRGB = true;
			coeffs.bandName = "RGB";
			// Store per-channel targets from config
			auto itR = gRadioRefs.find("0R");
			auto itG = gRadioRefs.find("0G");
			auto itB = gRadioRefs.find("0B");
			coeffs.targets_r = (itR != gRadioRefs.end()) ? itR->second.patches : defaultTargets;
			coeffs.targets_g = (itG != gRadioRefs.end()) ? itG->second.patches : defaultTargets;
			coeffs.targets_b = (itB != gRadioRefs.end()) ? itB->second.patches : defaultTargets;
		}
	}

	// Store single-band targets for multispectral images
	if (!coeffs.isRGB) {
		coeffs.targets = targets;
	}

	double stepX = (state.p3.x - state.p56.x) / 3.0;
	double stepY = (state.p3.y - state.p56.y) / 3.0;

	// Collect DN values for each channel
	vector<double> dns_r, dns_g, dns_b;

	for (int i = 0; i < 4; ++i) {
		int cx = cvRound(state.p56.x + i * stepX);
		int cy = cvRound(state.p56.y + i * stepY);

		Rect roi(cx - boxSize / 2, cy - boxSize / 2, boxSize, boxSize);
		// boundary check
		roi &= Rect(0, 0, img.cols, img.rows);

		if (roi.area() > 0) {
			Scalar avg = mean(img(roi));
			if (coeffs.isRGB && img.channels() >= 3) {
				// For RGB images, collect per-channel values (OpenCV uses BGR order)
				dns_b.push_back(avg[0]);
				dns_g.push_back(avg[1]);
				dns_r.push_back(avg[2]);
			} else {
				// For multispectral single-band images
				dns_r.push_back(avg[0]);
			}
			rectangle(display, roi, Scalar(0, 255, 0), 1);
		} else {
			dns_r.push_back(0);
			if (coeffs.isRGB) {
				dns_g.push_back(0);
				dns_b.push_back(0);
			}
		}
	}

	// Store DNS values in coeffs
	if (coeffs.isRGB) {
		coeffs.dns_r = dns_r;
		coeffs.dns_g = dns_g;
		coeffs.dns_b = dns_b;
	} else {
		coeffs.dns = dns_r;
	}

	
	// Save the image with markers for later review when auto-detect was used
	if (autoDetectThickness >= 0) {
		if (!exists(radioDir)) {
			create_directories(radioDir);
		}
		string savePath = radioDir + "/" + filename + "_board_preview.jpg";
		imwrite(savePath, display);
		gLog << "  Board preview saved to: " << savePath << endl;
	}
	
	// imshow(winName, display);
	// waitKey(500);
	// destroyWindow(winName);

	// Helper lambda for regression
	auto solveCoeffs = [](const vector<double>& dns, const vector<double>& tgts) -> pair<double, double> {
		Mat X(4, 2, CV_64F);
		Mat Y(4, 1, CV_64F);
		for (int i = 0; i < 4; ++i) {
			X.at<double>(i, 0) = dns[i];
			X.at<double>(i, 1) = 1.0;
			Y.at<double>(i, 0) = tgts[i];
		}
		Mat sol;
		solve(X, Y, sol, DECOMP_SVD);
		return {sol.at<double>(0, 0), sol.at<double>(1, 0)};
	};

	if (coeffs.isRGB && img.channels() >= 3) {
		// Calculate coefficients for each RGB channel
		auto [a_r, b_r] = solveCoeffs(dns_r, coeffs.targets_r);
		auto [a_g, b_g] = solveCoeffs(dns_g, coeffs.targets_g);
		auto [a_b, b_b] = solveCoeffs(dns_b, coeffs.targets_b);

		coeffs.a_r = a_r; coeffs.b_r = b_r;
		coeffs.a_g = a_g; coeffs.b_g = b_g;
		coeffs.a_b = a_b; coeffs.b_b = b_b;
		coeffs.valid = true;

		cout << "  RGB Coefficients calculated:" << endl;
		cout << "    Red:   a=" << a_r << ", b=" << b_r << endl;
		cout << "    Green: a=" << a_g << ", b=" << b_g << endl;
		cout << "    Blue:  a=" << a_b << ", b=" << b_b << endl;
	} else {
		// Single channel calibration
		auto [a, b] = solveCoeffs(dns_r, targets);
		coeffs.a = a;
		coeffs.b = b;
		coeffs.valid = true;

		cout << "  Coefficients calculated: a=" << coeffs.a << ", b=" << coeffs.b << endl;
	}

	// Store anchor points for collecting DN values from all images
	coeffs.p56 = state.p56;
	coeffs.p3 = state.p3;
	coeffs.boxSize = boxSize;

	return coeffs;
}

Mat applyRadiometricCalibration(const Mat& img, RadioCoeffs coeffs) {
	if (!coeffs.valid) return img.clone();

	Mat calibrated;

	if (coeffs.isRGB && img.channels() >= 3) {
		// For RGB images, apply per-channel calibration
		vector<Mat> channels(3);
		split(img, channels);

		// OpenCV uses BGR order
		channels[0].convertTo(channels[0], CV_64F, coeffs.a_b, coeffs.b_b); // Blue
		channels[1].convertTo(channels[1], CV_64F, coeffs.a_g, coeffs.b_g); // Green
		channels[2].convertTo(channels[2], CV_64F, coeffs.a_r, coeffs.b_r); // Red

		merge(channels, calibrated);
	} else {
		// For multispectral single-band images
		img.convertTo(calibrated, CV_64F, coeffs.a, coeffs.b);
	}

	// Normalizing to 0-255 based on min/max of the calibrated image (per Python logic)
	double minR, maxR;
	minMaxLoc(calibrated, &minR, &maxR);
	if (maxR == minR) maxR = minR + 1.0;

	Mat normalized;
	calibrated.convertTo(normalized, CV_8U, 255.0 / (maxR - minR), -minR * 255.0 / (maxR - minR));

	return normalized;
}

// Export radiometric calibration data to CSV
void exportRadiometricCsv(const string& outPath, const map<string, GroupData>& allGroups) {
	string csvPath = outPath + "/radiometric_report.csv";
	ofstream csv(csvPath);

	if (!csv.is_open()) {
		gLog << "  ERROR: Could not create CSV file: " << csvPath << endl;
		return;
	}

	// Set precision for floating-point output
	csv << fixed << setprecision(6);

	// Write header
	// Format: Filename,
	//         DN_B1_P1, DN_B1_P2, DN_B1_P3, DN_B1_P4,
	//         DN_B2_P1, DN_B2_P2, DN_B2_P3, DN_B2_P4,
	//         DN_B3_P1, DN_B3_P2, DN_B3_P3, DN_B3_P4,
	//         DN_B4_P1, DN_B4_P2, DN_B4_P3, DN_B4_P4,
	//         DN_B5_P1, DN_B5_P2, DN_B5_P3, DN_B5_P4,
	//         DN_RGB_R_P1, DN_RGB_R_P2, DN_RGB_R_P3, DN_RGB_R_P4,
	//         DN_RGB_G_P1, DN_RGB_G_P2, DN_RGB_G_P3, DN_RGB_G_P4,
	//         DN_RGB_B_P1, DN_RGB_B_P2, DN_RGB_B_P3, DN_RGB_B_P4,
	//         Slope (a), Intercept (b)
	csv << "Filename,";
	for (int band = 1; band <= 5; ++band) {
		csv << "DN_B" << band << "_P1,DN_B" << band << "_P2,DN_B" << band << "_P3,DN_B" << band << "_P4,";
	}
	csv << "DN_RGB_R_P1,DN_RGB_R_P2,DN_RGB_R_P3,DN_RGB_R_P4,";
	csv << "DN_RGB_G_P1,DN_RGB_G_P2,DN_RGB_G_P3,DN_RGB_G_P4,";
	csv << "DN_RGB_B_P1,DN_RGB_B_P2,DN_RGB_B_P3,DN_RGB_B_P4,";
	csv << "Slope (a),Intercept (b)" << endl;

	// Iterate through all groups
	for (const auto& [prefix, data] : allGroups) {
		if (!data.coeffs.valid) continue;

		// Write one row per group with all DN values
		csv << data.coeffs.filename << ",";

		// Write DN values for bands 1-5
		for (int band = 1; band <= 5; ++band) {
			auto it = data.multispectralDns.find(band);
			if (it != data.multispectralDns.end() && it->second.size() == 4) {
				csv << it->second[0] << "," << it->second[1] << ","
				    << it->second[2] << "," << it->second[3] << ",";
			} else {
				csv << "0,0,0,0,";
			}
		}

		// Write RGB DN values
		if (data.rgbCollected) {
			csv << data.rgbDns_r[0] << "," << data.rgbDns_r[1] << ","
			    << data.rgbDns_r[2] << "," << data.rgbDns_r[3] << ",";
			csv << data.rgbDns_g[0] << "," << data.rgbDns_g[1] << ","
			    << data.rgbDns_g[2] << "," << data.rgbDns_g[3] << ",";
			csv << data.rgbDns_b[0] << "," << data.rgbDns_b[1] << ","
			    << data.rgbDns_b[2] << "," << data.rgbDns_b[3] << ",";
		} else {
			csv << "0,0,0,0,0,0,0,0,0,0,0,0,";
		}

		// Write coefficients
		if (data.coeffs.isRGB) {
			// For RGB, use average of RGB coefficients
			double avgA = (data.coeffs.a_r + data.coeffs.a_g + data.coeffs.a_b) / 3.0;
			double avgB = (data.coeffs.b_r + data.coeffs.b_g + data.coeffs.b_b) / 3.0;
			csv << avgA << "," << avgB << endl;
		} else {
			csv << data.coeffs.a << "," << data.coeffs.b << endl;
		}
	}

	csv.close();
	gLog << "  Radiometric report exported to: " << csvPath << endl;
}

Mat contrastStretch(const Mat& src) {
	double minVal, maxVal;
	minMaxLoc(src, &minVal, &maxVal);
	
	Mat dst;
	if (maxVal > minVal) {
		src.convertTo(dst, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
	} else {
		src.convertTo(dst, CV_8U);
	}
	return dst;
}

Mat calculateNDVI(const Mat& red, const Mat& nir, const Mat& green_spectral_band = Mat(), const Mat& rgb_image = Mat(), const std::string& ndvi_output_dir = "", const std::string& debug_prefix = "") {
	Mat r_float, n_float;
	red.convertTo(r_float, CV_32F);
	nir.convertTo(n_float, CV_32F);

	Mat ndvi_top = n_float - r_float;
	Mat ndvi_bottom = n_float + r_float;
	
	// Avoid division by zero
	Mat mask = (ndvi_bottom == 0);
	ndvi_bottom.setTo(0.00001f, mask);

	Mat ndvi = ndvi_top / ndvi_bottom;
	
	// Clip to [0, 1] as per python script reference
	threshold(ndvi, ndvi, 0, 0, THRESH_TOZERO);
	threshold(ndvi, ndvi, 1, 1, THRESH_TRUNC);
	
	gLog << "    ndvi_output_dir: " << ndvi_output_dir << endl;
	if (!rgb_image.empty()) {
		gLog << "    !rgb_image.empty" << endl;

		// Convert BGR to HSV
		Mat hsv;
		cvtColor(rgb_image, hsv, COLOR_BGR2HSV);

		// Define range of green color in HSV (similar to green_zone_detection.py)
		// These values might need fine-tuning based on actual image data
		// Lower_green = [40, 50, 50]
		// Upper_green = [80, 255, 255]
		Scalar lower_green = Scalar(40, 40, 40);
		Scalar upper_green = Scalar(90, 255, 255);

		// Threshold the HSV image to get only green colors
		Mat green_mask;
		inRange(hsv, lower_green, upper_green, green_mask);

		// Mask out non-green areas (set to 0) in the NDVI image
		// Ensure the green_mask is of the same size as ndvi
		if (ndvi.size() == green_mask.size()) {
			ndvi.setTo(0, green_mask == 0);
		} else {
			// If sizes don't match, we might need to resize or handle error
			// For now, print a warning. In a real scenario, proper resizing or error handling would be needed.
			cerr << "Warning: RGB image mask size does not match NDVI image size. Green zone masking skipped." << endl;
		}

		green_mask.convertTo(green_mask, CV_8U);
		if (!ndvi_output_dir.empty()) {
			imwrite(ndvi_output_dir + "/" + debug_prefix + "green_mask.jpg", green_mask);
		} else {
			imwrite(debug_prefix + "green_mask.jpg", green_mask);
		}
	}

	return ndvi;
}

void showUsage() {
	cout << "USAGE: ./calib <src_dir (default: .input/)> <dest_dir (default: .output/)> [--radio] [--auto] [--optimize]" << endl;
	cout << "  --radio       Enable radiometric calibration." << endl;
	cout << "  --auto        Auto-detect radiometric board (used with --radio). Optional: --auto <border_thickness>" << endl;
	cout << "  --template    Path to radiometric board template image (default: .input_ref/radiometric_board.jpg)" << endl;
	cout << "  --ref         Path to radiometric reference CSV file (default: .input_ref/radiometric_reference.csv)" << endl;
	cout << "  --optimize    Enable performance optimizations for ECC alignment." << endl;
	cout << "                Optional: --optimize <downscale>,<iterations>,<epsilon>" << endl;
	cout << "                Default: --optimize 2,50,1e-4" << endl;
#ifdef WINGUI
	cout << "  --gui         Launch Windows GUI interface." << endl;
#endif
	cout << "" << endl;
	cout << "---" << endl;
	cout << "" << endl;
}

int main(int argc, char** argv) {
	// Generate log filename
	auto t = time(nullptr);
	auto tm = *localtime(&t);
	ostringstream oss;
	oss << put_time(&tm, "%y%m%d-%H%M") << ".log";
	
	// Ensure .logs directory exists
	string logDir = ".logs";
	if (!exists(logDir)) {
		create_directories(logDir);
	}
	
	gLog.open(logDir + "/" + oss.str());

	string inDir = ".input";
	string outDir = ".output";
	string radioRefFile = ".input_ref/radiometric_reference.csv";
	string radioTemplatePath = ".input_ref/radiometric_board.jpg";
	bool doRadio = false;
	int autoRadioThickness = -1;
	Point radioInterval(40, 0);

	// Check for GUI flag first
	gLog << "argv[1]: " << argv[1] << endl;

	#ifdef WINGUI
	bool useGui = true;
	if (argc > 2) {
		useGui = false;
	}
	#else
	bool useGui = false;
	if (argc > 1 && string(argv[1]) == "--gui") {
		useGui = true;
	}
	#endif

	gLog << "useGui: " << useGui << endl;

	if (useGui) {
		#ifdef WINGUI
		gLog << "Launching GUI..." << endl;
		
		bool guiTwoPointClick = false;
		bool guiAutoDetect = false;
		int guiBoardThickness = 10;
		string guiTemplatePath = radioTemplatePath;

		gConfig.ecc.enabled = true; // Enabled by default in CLI
		if (!runCalibGui(inDir, outDir, radioRefFile, doRadio, guiTwoPointClick, guiAutoDetect, guiBoardThickness, guiTemplatePath)) {
			gLog << "GUI cancelled or exited." << endl;
			return 0;
		}
		
		// Apply GUI settings
		if (guiAutoDetect) {
			autoRadioThickness = (guiBoardThickness == 0) ? 0 : guiBoardThickness;
		}
		if (guiTwoPointClick) {
			autoRadioThickness = -1; // Disable auto-detect, use manual 2-point click
		}
		radioTemplatePath = guiTemplatePath;
		
		gLog << "GUI Configuration:" << endl;
		gLog << "  Input Folder: " << inDir << endl;
		gLog << "  Output Folder: " << outDir << endl;
		gLog << "  Radiometric Ref: " << radioRefFile << endl;
		gLog << "  Radiometric Template: " << radioTemplatePath << endl;
		gLog << "  Radiometric Calibration: " << (doRadio ? "ENABLED" : "disabled") << endl;
		gLog << "  Auto-Detect Board: " << (guiAutoDetect ? "ENABLED" : "disabled") << endl;
		if (guiAutoDetect) {
			gLog << "  Board Thickness: " << autoRadioThickness << endl;
		}
		gLog << endl;
		
		#else
		cerr << "Error: GUI support not compiled. Define WINGUI and compile with MinGW-w64." << endl;
		return 1;
		#endif
	} else {
		// Original CLI argument parsing
		if (argc == 1) {
			showUsage();
		}

		gConfig.ecc.enabled = false; // Disabled by default in CLI

		vector<string> args;
		for (int i = 1; i < argc; ++i) {
			string arg = argv[i];
			if (arg == "--radio") {
				doRadio = true;
				if (i + 1 < argc && argv[i+1][0] != '-') {
					string nextArg = argv[i+1];
					size_t comma = nextArg.find(',');
					if (comma != string::npos) {
						radioInterval.x = stoi(nextArg.substr(0, comma));
						radioInterval.y = stoi(nextArg.substr(comma + 1));
						i++;
					}
				}
			} else if (arg == "--optimize") {
				gConfig.ecc.enabled = true;
				if (i + 1 < argc && argv[i+1][0] != '-') {
					string optStr = argv[i+1];
					stringstream ss(optStr);
					string segment;
					vector<string> parts;
					while(getline(ss, segment, ',')) parts.push_back(segment);
					
					if (parts.size() >= 1) gConfig.ecc.downscaleFactor = stoi(parts[0]);
					if (parts.size() >= 2) gConfig.ecc.maxIterations = stoi(parts[1]);
					if (parts.size() >= 3) gConfig.ecc.epsilon = stod(parts[2]);
					i++;
				}
			} else if (arg == "--auto") {
				autoRadioThickness = 0;
				if (i + 1 < argc && isdigit(argv[i+1][0])) {
					autoRadioThickness = stoi(argv[i+1]);
					i++;
				}
			} else if (arg == "--template") {
				if (i + 1 < argc && argv[i+1][0] != '-') {
					radioTemplatePath = argv[i+1];
					i++;
				}
			} else if (arg == "--ref") {
				if (i + 1 < argc && argv[i+1][0] != '-') {
					radioRefFile = argv[i+1];
					i++;
				}
			} else {
				args.push_back(arg);
			}
		}

		if (args.size() > 0) inDir = args[0];
		if (args.size() > 1) outDir = args[1];
	}

	// Create directories if they don't exist
	if (!exists(inDir)) {
		create_directories(inDir);
		gLog << "Created input directory: " << inDir << endl;
	}
	if (!exists(outDir)) {
		create_directories(outDir);
		gLog << "Created output directory: " << outDir << endl;
	}

	// Create subdirectories for each processing step
	string calibDir = outDir + "/calib";
	string radioDir = outDir + "/radio";
	string ndviDir = outDir + "/ndvi";

	create_directories(calibDir);
	create_directories(radioDir);
	create_directories(ndviDir);

	gLog << "Calibrated images: " << calibDir << endl;
	gLog << "Radiometric images: " << radioDir << endl;
	gLog << "NDVI images: " << ndviDir << endl;
	gLog << "========================================" << endl;
	gLog << "UAV Calibration running" << endl;
	gLog << "Time: " << oss.str().substr(0, 11) << endl;
	if (doRadio) {
		gLog << "Radiometric calibration ENABLED with interval (" << radioInterval.x << "," << radioInterval.y << ")" << (autoRadioThickness >= 0 ? " [AUTO]" : "") << endl;
		if (autoRadioThickness > 0) gLog << "  Auto-detect border thickness: " << autoRadioThickness << endl;
	}
	gLog << "Input: " << inDir << endl;
	gLog << "Output: " << outDir << endl;
	gLog << "========================================" << endl << endl;

	// Step 1: Scan and Parse Metadata for all groups
	map<string, GroupData> allGroups;

	gLog << "Scanning " << inDir << "..." << endl;
	for (const auto& entry : directory_iterator(inDir)) {
		string path = entry.path().string();
		string stem = entry.path().stem().string();
		string ext = entry.path().extension().string();

		if (ext != ".tif" && ext != ".TIF" && ext != ".jpg" && ext != ".JPG") continue;
		if (stem.empty()) continue;

		string prefix = stem.substr(0, stem.length() - 1);
		allGroups[prefix].prefix = prefix;
		allGroups[prefix].images.push_back(parseMetadata(path));
	}

	// Identify reference images for each group
	for (auto& [prefix, data] : allGroups) {
		for (auto& info : data.images) {
			if (abs(info.relX) < 0.001 && abs(info.relY) < 0.001) {
				data.refInfo = &info;
				break;
			}
		}
	}

	// Step 2: Collect Radiometric Calibration inputs for ALL groups upfront
	if (doRadio) {
		gLog << "\n--- RADIOMETRIC CALIBRATION PHASE ---" << endl;

		// Load radiometric reference data from config file
		loadRadiometricRefs(radioRefFile);
		
		for (auto& [prefix, data] : allGroups) {
			ImageInfo* targetInfo = data.refInfo;
			if (!targetInfo && !data.images.empty()) {
				targetInfo = &data.images[0];
			}

			if (targetInfo) {
				Mat raw = imread(targetInfo->path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
				if (!raw.empty()) {
					data.coeffs = getRadiometricCoeffs(raw, targetInfo->filename, radioInterval, autoRadioThickness, radioDir, radioTemplatePath);
					
					// Store anchor points from the reference image
					Point p56 = data.coeffs.p56;
					Point p3 = data.coeffs.p3;
					int boxSize = data.coeffs.boxSize;
					bool isRGB = data.coeffs.isRGB;
					
					if (data.coeffs.valid) {
						gLog << "  Collecting DN values from all images in group..." << endl;
						
						// Collect DN values from all images in the group
						for (auto& info : data.images) {
							Mat img = imread(info.path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
							if (img.empty()) continue;
							
							string stem = path(info.filename).stem().string();
							if (stem.empty()) continue;
							char lastChar = stem.back();
							
							if (lastChar == '0') {
								// RGB image - collect per-channel DN values
								vector<double> dns_r, dns_g, dns_b;
								collectDnValues(img, p56, p3, boxSize, true, dns_r, dns_g, dns_b);
								data.rgbDns_r = dns_r;
								data.rgbDns_g = dns_g;
								data.rgbDns_b = dns_b;
								data.rgbCollected = true;
								gLog << "    Collected RGB DN values from " << info.filename << endl;
							} else {
								// Multispectral image - collect single-band DN values
								int band = lastChar - '0';
								vector<double> dns_r, dns_g, dns_b;
								collectDnValues(img, p56, p3, boxSize, false, dns_r, dns_g, dns_b);
								data.multispectralDns[band] = dns_r;
								gLog << "    Collected band " << band << " DN values from " << info.filename << endl;
							}
						}
					}
				}
			}
		}
		gLog << "--- CALIBRATION PHASE COMPLETE ---\n" << endl;

		// Export radiometric calibration data to CSV
		gLog << "--- EXPORTING RADIOMETRIC CSV ---" << endl;
		exportRadiometricCsv(outDir, allGroups);
		gLog << "--- CSV EXPORT COMPLETE ---\n" << endl;
	}

	// Step 3: Process all groups
	vector<string> prefixes;
	for (auto const& [prefix, data] : allGroups) {
		prefixes.push_back(prefix);
	}

	gLog << "\n" << endl;

	for (int i = 0; i < (int)prefixes.size(); ++i) {
		string prefix = prefixes[i];
		GroupData& data = allGroups[prefix];

		gLog << "****************************************" << endl;
		gLog << "Processing group: " << prefix << " (" << data.images.size() << " images)" << endl;
		gLog << "****************************************" << endl;

		Mat refMat;
		if (data.refInfo) {
			gLog << "  Reference found: " << data.refInfo->filename << endl;
			Mat rawRef = imread(data.refInfo->path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
			if (!rawRef.empty()) {
				Mat processedRef = rawRef;
				if (doRadio && data.coeffs.valid) {
					processedRef = applyRadiometricCalibration(rawRef, data.coeffs);
				}
				refMat = undistortImg(processedRef, *data.refInfo);
			}
		} else {
			gLog << "  No reference image found for group " << prefix << endl;
		}

		map<int, Mat> alignedBands;

		#pragma omp parallel for schedule(dynamic)
		for (int j = 0; j < (int)data.images.size(); ++j) {
			auto& info = data.images[j];
			stringstream ss;
			ss << "\n  [Image: " << info.filename << "]" << endl;

			Mat raw = imread(info.path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
			if (raw.empty()) {
				ss << "    Error: could not load " << info.path << endl;
				gLog << ss.str();
				continue;
			}

			Mat processed = raw;
			if (doRadio && data.coeffs.valid) {
				processed = applyRadiometricCalibration(raw, data.coeffs);
			}

			// --- STEP A: DEWARP ALIGNMENT (Metadata) ---
			ss << "    Step A: Dewarping..." << endl;
			Mat dewarped = undistortImg(processed, info);
			Mat finalImg;

			// --- STEP B: INITIAL ALIGNMENT (Metadata) ---
			Mat H_meta = Mat::eye(3, 3, CV_64F);
			if (info.foundH) {
				ss << "    Step B: Applying H_meta" << endl;
				H_meta = info.H;
			} else if (abs(info.relX) > 0.0001 || abs(info.relY) > 0.0001) {
				// Translation
				ss << "    Step B: Translation (" << info.relX << ", " << info.relY << ")" << endl;
				H_meta.at<double>(0, 2) = info.relX;
				H_meta.at<double>(1, 2) = info.relY;
			}

			// --- STEP C: OPTIONAL FINE TUNING (ECC + SIFT Fallback) ---
			Mat H_total = H_meta.clone();
			Size finalSize = dewarped.size();

			if (data.refInfo && data.refInfo->path != info.path && !refMat.empty()) {
				ss << "    Step C: Alignment to " << data.refInfo->filename << "..." << endl;
				clock_t startC = clock();

				// 1. RESOLUTION MATCHING: Prepare metadata transform for reference resolution
				double scaleX = (double)refMat.cols / dewarped.cols;
				double scaleY = (double)refMat.rows / dewarped.rows;

				Mat H_meta_ref = H_meta.clone();
				// Column 0 and 1 map output coordinates to input coordinates.
				// Since output is scaled down, input coords must be scaled up.
				H_meta_ref.col(0) *= (1.0 / scaleX);
				H_meta_ref.col(1) *= (1.0 / scaleY);

				// Apply metadata warp to get close, at reference resolution
				Mat alignedMeta;
				clock_t tWarp = clock();
				warpPerspective(dewarped, alignedMeta, H_meta_ref, refMat.size(), INTER_LINEAR | WARP_INVERSE_MAP);
				ss << "      - Metadata warp: " << fixed << setprecision(2) << double(clock() - tWarp) / CLOCKS_PER_SEC << "s" << endl;
				finalSize = refMat.size();

				// 2. Prepare images for fine alignment
				clock_t tPrep = clock();
				Mat alignedGray = prepareForECC(alignedMeta);
				Mat refGray = prepareForECC(refMat);

				// ECC Optimization: Downscale images if enabled
				Mat eccRef = refGray;
				Mat eccAligned = alignedGray;
				float eccScale = 1.0f;

				if (gConfig.ecc.enabled && gConfig.ecc.downscaleFactor > 1) {
					eccScale = 1.0f / gConfig.ecc.downscaleFactor;
					resize(refGray, eccRef, Size(), eccScale, eccScale, INTER_AREA);
					resize(alignedGray, eccAligned, Size(), eccScale, eccScale, INTER_AREA);
					ss << "      - Optimization: Downscaled images by " << gConfig.ecc.downscaleFactor << "x" << endl;
				}
				ss << "      - Image preparation: " << fixed << setprecision(2) << double(clock() - tPrep) / CLOCKS_PER_SEC << "s" << endl;

				// 3. Try ECC (Primary Method)
				int motionType = MOTION_HOMOGRAPHY;
				Mat H_ecc = Mat::eye(3, 3, CV_32F);
				
				// Configure criteria based on optimization
				int maxIter = gConfig.ecc.enabled ? gConfig.ecc.maxIterations : 500;
				double eps = gConfig.ecc.enabled ? gConfig.ecc.epsilon : 1e-6;
				TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, maxIter, eps);

				bool aligned = false;
				clock_t tECC = clock();
				try {
					double cc = findTransformECC(eccRef, eccAligned, H_ecc, motionType, criteria);
					
					// If we downscaled, we need to adjust the resulting Homography matrix
					if (eccScale != 1.0f) {
						// H_full = S_inv * H_down * S
						// where S is the scaling matrix: [s 0 0; 0 s 0; 0 0 1]
						// S_inv is: [1/s 0 0; 0 1/s 0; 0 0 1]
						// For Homography, this means scaling the translation components (H02, H12) 
						// and the perspective components (H20, H21) accordingly.
						H_ecc.at<float>(0, 2) /= eccScale;
						H_ecc.at<float>(1, 2) /= eccScale;
						H_ecc.at<float>(2, 0) *= eccScale;
						H_ecc.at<float>(2, 1) *= eccScale;
					}

					ss << "      - ECC converged (cc=" << cc << ", took " << fixed << setprecision(2) << double(clock() - tECC) / CLOCKS_PER_SEC << "s)" << endl;
					aligned = true;
				} catch (const cv::Exception& e) {
					ss << "      - ECC failed after " << fixed << setprecision(2) << double(clock() - tECC) / CLOCKS_PER_SEC << "s. Falling back to SIFT..." << endl;

					// 4. SIFT Fallback (Safety Net)
					clock_t tSIFT = clock();
					Mat refClahe = getCLAHE(refMat);
					Mat alignedClahe = getCLAHE(alignedMeta);
					Mat H_sift = alignSIFTFallback(refClahe, alignedClahe);

					if (!H_sift.empty()) {
						H_sift.convertTo(H_ecc, CV_32F);
						ss << "      - SIFT alignment successful (took " << fixed << setprecision(2) << double(clock() - tSIFT) / CLOCKS_PER_SEC << "s)" << endl;
						aligned = true;
					} else {
						ss << "      - CRITICAL: Both ECC and SIFT failed (SIFT took " << fixed << setprecision(2) << double(clock() - tSIFT) / CLOCKS_PER_SEC << "s)" << endl;
					}
				}

				if (aligned) {
					Mat H_ecc_64F;
					H_ecc.convertTo(H_ecc_64F, CV_64F);
					H_total = H_meta_ref * H_ecc_64F;
				} else {
					H_total = H_meta_ref; // Fallback to metadata only
				}
				ss << "      - Step C Total: " << fixed << setprecision(2) << double(clock() - startC) / CLOCKS_PER_SEC << "s" << endl;
			}

			ss << "    H_total: " << H_total << endl;
			ss << "    Saving " << info.filename << "..." << endl;

			Mat tmpImg;

			clock_t tFinalWarp = clock();
			warpPerspective(dewarped, finalImg, H_total, finalSize, INTER_LINEAR | WARP_INVERSE_MAP);

			tmpImg = contrastStretch(finalImg);
			imwrite(calibDir + "/" + info.filename + ".jpg", tmpImg);

			ss << "      - Final warp & save: " << fixed << setprecision(2) << double(clock() - tFinalWarp) / CLOCKS_PER_SEC << "s" << endl;

			// Save radiometric calibrated image if radio is enabled
			if (doRadio && data.coeffs.valid) {
				clock_t tRadio = clock();
				Mat radioImg = applyRadiometricCalibration(raw, data.coeffs);
				radioImg = undistortImg(radioImg, info);
				warpPerspective(radioImg, radioImg, H_total, finalSize, INTER_LINEAR | WARP_INVERSE_MAP);

				tmpImg = contrastStretch(radioImg);
				imwrite(radioDir + "/" + info.filename + ".jpg", tmpImg);

				finalImg = radioImg.clone();

				ss << "      - Radiometric processing: " << fixed << setprecision(2) << double(clock() - tRadio) / CLOCKS_PER_SEC << "s" << endl;
			}

			// Identify bands for NDVI (last char of stem, e.g. DJI_0223.TIF -> 2=Green, 3=Red, 5=NIR, 0=RGB)
			string stem = path(info.path).stem().string();
			if (!stem.empty()) {
				int band = stem.back() - '0';
				if (band == 0 || band == 1 || band == 2 || band == 3 || band == 4 || band == 5) {
					#pragma omp critical
					{
						alignedBands[band] = finalImg.clone();
					}
				}
			}
			gLog << ss.str();
		}

		// Calculate and save NDVI
		if (alignedBands.count(3) && alignedBands.count(5)) {
			gLog << "\n    Step D: Calculating NDVI for group " << prefix << "..." << endl;
			clock_t startD = clock();

			Mat ndvi = calculateNDVI(alignedBands[3], alignedBands[5], alignedBands[2], alignedBands[0], ndviDir, prefix);

			// Save raw float NDVI
			imwrite(ndviDir + "/" + prefix + "ndvi_raw.tif", ndvi);

			// Save colorized NDVI
			Mat ndvi_u8 = contrastStretch(ndvi);
			imwrite(ndviDir + "/" + prefix + "ndvi_u8.jpg", ndvi_u8);

			Mat ndvi_color;
			applyColorMap(ndvi_u8, ndvi_color, COLORMAP_JET);
			imwrite(ndviDir + "/" + prefix + "ndvi_color.jpg", ndvi_color);

			gLog << "    NDVI saved to " << prefix + "ndvi_raw.tif and " << prefix + "ndvi_color.jpg" << endl;
			gLog << "    Step D Total: " << fixed << setprecision(2) << double(clock() - startD) / CLOCKS_PER_SEC << "s" << endl;
		}
	}

	if (useGui) {
		cout << "Press ENTER to exit..." << endl;
		cin.get();
	}

	return 0;
}