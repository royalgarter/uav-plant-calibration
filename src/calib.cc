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
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp> // For findTransformECC
#include <opencv2/highgui.hpp>
#include <tiffio.h>

using namespace std;
using namespace std::filesystem;
using namespace cv;

// --- LOGGING UTILITY ---
struct Logger {
	ofstream file;
	bool fileOpen = false;

	void open(const string& filename) {
		file.open(filename);
		fileOpen = file.is_open();
	}

	template<typename T>
	Logger& operator<<(const T& msg) {
		cout << msg;
		if (fileOpen) file << msg;
		return *this;
	}

	// For endl/iomanip
	Logger& operator<<(ostream& (*f)(ostream&)) {
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

RadioCoeffs getRadiometricCoeffs(const Mat& img, const string& filename, Point interval, bool autoDetect = false) {
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

	if (autoDetect) {
		string templatePath = "example/calib/radiometric_board.jpg";
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
				// We need to account for rotation.
				Point p56_rel, p3_rel;
				if (bestRot == 0) { // Vertical: 56% top, 3% bottom
					p56_rel = Point(bestSize.width / 2, bestSize.height / 8);
					p3_rel = Point(bestSize.width / 2, bestSize.height * 7 / 8);
				} else if (bestRot == 1) { // 90 deg: 56% right, 3% left? No, let's be careful.
					// Vertical Template (T) -> Rotate 90 CW -> Horizontal (H)
					// T(w, h) -> H(h, w)
					// Top of T (p56) becomes Right of H.
					p56_rel = Point(bestSize.width * 7 / 8, bestSize.height / 2);
					p3_rel = Point(bestSize.width / 8, bestSize.height / 2);
				} else if (bestRot == 2) { // 180 deg: 56% bottom, 3% top
					p56_rel = Point(bestSize.width / 2, bestSize.height * 7 / 8);
					p3_rel = Point(bestSize.width / 2, bestSize.height / 8);
				} else if (bestRot == 3) { // 270 deg: 56% left, 3% right
					p56_rel = Point(bestSize.width / 8, bestSize.height / 2);
					p3_rel = Point(bestSize.width * 7 / 8, bestSize.height / 2);
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

	// Reference Reflectances from Calibration Target Surface Reflectance and Camera Response
	// Identify band for multispectral camera image using last character of file name
	vector<double> targets_5nir = {0.5647, 0.3582, 0.1148, 0.0272}; // NIR Near-infared image No.5
	vector<double> targets_4re = {0.5618, 0.3666, 0.1191, 0.0267}; // RE Red edge image No.4
	vector<double> targets_3red = {0.5599, 0.3740, 0.1228, 0.0262}; // Red image No.3
	vector<double> targets_2green = {0.5567, 0.3835, 0.1256, 0.0256}; // Green image No.2
	vector<double> targets_1blue = {0.5500704789, 0.390806116, 0.126676427, 0.02539390723}; // Blue image No.1
	vector<double> targets_0rgb_red = {0.5581634717, 0.3790530195, 0.124637386, 0.02591108772}; // Red channel of RGB image No.0
	vector<double> targets_0rgb_green = {0.5554092039, 0.3851975973, 0.1255882691, 0.02554883719}; // Green channel of RGB image No.0
	vector<double> targets_0rgb_blue = {0.5500704789, 0.390806116, 0.126676427, 0.02539390723}; // Blue channel of RGB image No.0

	// Select targets based on spectral band (last character of filename stem)
	vector<double> targets = targets_5nir; // default to NIR
	string stem = path(filename).stem().string();
	if (!stem.empty()) {
		char lastChar = stem.back();
		if (lastChar == '5') targets = targets_5nir;
		else if (lastChar == '4') targets = targets_4re;
		else if (lastChar == '3') targets = targets_3red;
		else if (lastChar == '2') targets = targets_2green;
		else if (lastChar == '1') targets = targets_1blue;
		else if (lastChar == '0') {
			// For RGB image (No.0), calibrate each channel separately
			coeffs.isRGB = true;
		}
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

	imshow(winName, display);
	waitKey(500);
	destroyWindow(winName);

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
		auto [a_r, b_r] = solveCoeffs(dns_r, targets_0rgb_red);
		auto [a_g, b_g] = solveCoeffs(dns_g, targets_0rgb_green);
		auto [a_b, b_b] = solveCoeffs(dns_b, targets_0rgb_blue);

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

Mat calculateNDVI(const Mat& red, const Mat& nir) {
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
	
	return ndvi;
}


void showUsage() {
	cout << "USAGE: ./calib <src_dir (default: .input/)> <dest_dir (default: .output/)> [--radio] [--auto]" << endl;
	cout << "  --radio       Enable radiometric calibration." << endl;
	cout << "  --auto        Auto-detect radiometric board (used with --radio)." << endl;
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
	gLog.open(oss.str());

	string inDir = ".input";
	string outDir = ".output";
	bool doRadio = false;
	bool autoRadio = false;
	Point radioInterval(40, 0);

	if (argc == 1) {
		showUsage();
	}

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
		} else if (arg == "--auto") {
			autoRadio = true;
		} else {
			args.push_back(arg);
		}
	}

	if (args.size() > 0) inDir = args[0];
	if (args.size() > 1) outDir = args[1];

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
	if (doRadio) gLog << "Radiometric calibration ENABLED with interval (" << radioInterval.x << "," << radioInterval.y << ")" << (autoRadio ? " [AUTO]" : "") << endl;
	gLog << "Input: " << inDir << endl;
	gLog << "Output: " << outDir << endl;
	gLog << "========================================" << endl << endl;

	// Step 1: Scan and Parse Metadata for all groups
	struct GroupData {
		string prefix;
		vector<ImageInfo> images;
		ImageInfo* refInfo = nullptr;
		RadioCoeffs coeffs;
	};
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
		for (auto& [prefix, data] : allGroups) {
			ImageInfo* targetInfo = data.refInfo;
			if (!targetInfo && !data.images.empty()) {
				targetInfo = &data.images[0];
			}

			if (targetInfo) {
				Mat raw = imread(targetInfo->path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
				if (!raw.empty()) {
					data.coeffs = getRadiometricCoeffs(raw, targetInfo->filename, radioInterval, autoRadio);
				}
			}
		}
		gLog << "--- CALIBRATION PHASE COMPLETE ---\n" << endl;
	}

	// Step 3: Process all groups
	for (auto& [prefix, data] : allGroups) {
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

		for (auto& info : data.images) {
			gLog << "\n  [Image: " << info.filename << "]" << endl;

			Mat raw = imread(info.path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
			if (raw.empty()) {
				gLog << "    Error: could not load " << info.path << endl;
				continue;
			}

			Mat processed = raw;
			if (doRadio && data.coeffs.valid) {
				processed = applyRadiometricCalibration(raw, data.coeffs);
			}

			// --- STEP A: DEWARP ALIGNMENT (Metadata) ---
			gLog << "    Step A: Dewarping..." << endl;
			Mat dewarped = undistortImg(processed, info);
			Mat finalImg;

			// --- STEP B: INITIAL ALIGNMENT (Metadata) ---
			Mat H_meta = Mat::eye(3, 3, CV_64F);
			if (info.foundH) {
				gLog << "    Step B: Applying H_meta" << endl;
				H_meta = info.H;
			} else if (abs(info.relX) > 0.0001 || abs(info.relY) > 0.0001) {
				// Translation
				gLog << "    Step B: Translation (" << info.relX << ", " << info.relY << ")" << endl;
				H_meta.at<double>(0, 2) = info.relX;
				H_meta.at<double>(1, 2) = info.relY;
			}

			// --- STEP C: OPTIONAL FINE TUNING (ECC) ---
			Mat H_total = H_meta.clone();

			if (data.refInfo && data.refInfo->path != info.path && !refMat.empty()) {
				gLog << "    Step C: ECC Fine Alignment to " << data.refInfo->filename << "..." << endl;

				// 1. Apply metadata warp first to get close
				Mat alignedMeta;
				warpPerspective(dewarped, alignedMeta, H_meta, dewarped.size(), INTER_LINEAR | WARP_INVERSE_MAP);

				// 2. Prepare images for ECC
				Mat alignedGray, refGray;
				if (alignedMeta.channels() > 1) cvtColor(alignedMeta, alignedGray, COLOR_BGR2GRAY);
				else alignedGray = alignedMeta.clone();

				if (refMat.channels() > 1) cvtColor(refMat, refGray, COLOR_BGR2GRAY);
				else refGray = refMat.clone();

				// Convert to CV_32F for ECC (required: 8U or 32F)
				if (alignedGray.depth() != CV_32F) alignedGray.convertTo(alignedGray, CV_32F);
				if (refGray.depth() != CV_32F) refGray.convertTo(refGray, CV_32F);

				// Optional: Normalize to 0-1 range for better numerical stability with ECC
				normalize(alignedGray, alignedGray, 0, 1, NORM_MINMAX);
				normalize(refGray, refGray, 0, 1, NORM_MINMAX);

				// 3. Run ECC

				// Old Affine conversion logic
				// int motionType = MOTION_AFFINE;
				// Mat H_ecc = Mat::eye(2, 3, CV_32F);

				// New Homography
				int motionType = MOTION_HOMOGRAPHY;
				Mat H_ecc = Mat::eye(3, 3, CV_32F);

				TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, 50, 1e-3);

				try {
					double cc = findTransformECC(refGray, alignedGray, H_ecc, motionType, criteria);
					gLog << "    ECC converged (cc=" << cc << ")" << endl;

					gLog << "    H_ecc: " << H_ecc << endl;

					// 4. Compose transforms
					// H_meta maps: Dst (Aligned) -> Src (Original)
					// H_ecc maps: Dst (Ref) -> Src (Aligned)  [Backward mapping for WARP_INVERSE_MAP]
					// We want: Ref -> Original
					// H_total = H_meta * H_ecc

					Mat H_ecc_64F;
					H_ecc.convertTo(H_ecc_64F, CV_64F);

					// Old Affine conversion logic
					Mat H_ecc_3x3 = Mat::eye(3, 3, CV_64F);
					H_ecc_3x3.at<double>(0,0) = H_ecc.at<float>(0,0);
					H_ecc_3x3.at<double>(0,1) = H_ecc.at<float>(0,1);
					H_ecc_3x3.at<double>(0,2) = H_ecc.at<float>(0,2);
					H_ecc_3x3.at<double>(1,0) = H_ecc.at<float>(1,0);
					H_ecc_3x3.at<double>(1,1) = H_ecc.at<float>(1,1);
					H_ecc_3x3.at<double>(1,2) = H_ecc.at<float>(1,2);
					H_total = H_meta * H_ecc_3x3;

					// New Homography
					H_total = H_meta * H_ecc_64F;

				} catch (const cv::Exception& e) {
					gLog << "    ECC failed: " << e.what() << endl;
				}
			}

			gLog << "    H_total: " << H_total << endl;
			gLog << "    Saving " << info.filename << endl;

			warpPerspective(dewarped, finalImg, H_total, dewarped.size(), INTER_LINEAR | WARP_INVERSE_MAP);
			imwrite(calibDir + "/" + info.filename, finalImg);

			// Save radiometric calibrated image if radio is enabled
			if (doRadio && data.coeffs.valid) {
				Mat radioImg = applyRadiometricCalibration(raw, data.coeffs);
				radioImg = undistortImg(radioImg, info);
				warpPerspective(radioImg, radioImg, H_total, radioImg.size(), INTER_LINEAR | WARP_INVERSE_MAP);
				imwrite(radioDir + "/" + info.filename, radioImg);
			}

			// Identify band for NDVI (last char of stem, e.g. DJI_0223.TIF -> 3=Red, 5=NIR)
			string stem = path(info.path).stem().string();
			if (!stem.empty()) {
				int band = stem.back() - '0';
				if (band == 3 || band == 5) {
					alignedBands[band] = finalImg.clone();
				}
			}
		}

		// Calculate and save NDVI
		if (alignedBands.count(3) && alignedBands.count(5)) {
			gLog << "\n    Step D: Calculating NDVI for group " << prefix << "..." << endl;
			Mat ndvi = calculateNDVI(alignedBands[3], alignedBands[5]);

			// Save raw float NDVI
			imwrite(ndviDir + "/" + prefix + "NDVI_raw.tif", ndvi);

			// Save colorized NDVI
			Mat ndvi_u8 = contrastStretch(ndvi);
			Mat ndvi_color;
			applyColorMap(ndvi_u8, ndvi_color, COLORMAP_JET);
			imwrite(ndviDir + "/" + prefix + "NDVI_color.jpg", ndvi_color);
			gLog << "    NDVI saved to " << prefix + "NDVI_raw.tif and " << prefix + "NDVI_color.jpg" << endl;
		}
	}

	cout << "Press ENTER to exit..." << endl;
	cin.get();

	return 0;
}