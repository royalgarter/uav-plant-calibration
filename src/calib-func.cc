#include "calib.h"
#include <iomanip>
#include <ctime>
#include <regex>
#include <sstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <tiffio.h>

using namespace std;
using namespace cv;
using namespace std::filesystem;

// --- GLOBALS DEFINITION ---
Logger gLog;
Config gConfig;
map<string, RadioRef> gRadioRefs;

// --- METADATA ---

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

// --- IMAGE PROCESSING ---

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
		Mat H = findHomography(dst_pts, src_pts, RANSAC);
		return H;
	}

	return Mat();
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

// --- RADIOMETRIC ---

bool loadRadiometricRefs(const string& path) {
	ifstream file(path);
	if (!file.is_open()) {
		gLog << "  WARNING: Could not load radiometric reference file: " << path << endl;
		return false;
	}

	string line;
	while (getline(file, line)) {
		if (line.empty() || line[0] == '#') continue;
		if (line.find("band,") == 0) continue;

		stringstream ss(line);
		string band, p56, p36, p12, p3;
		getline(ss, band, ',');
		getline(ss, p56, ',');
		getline(ss, p36, ',');
		getline(ss, p12, ',');
		getline(ss, p3, ',');

		RadioRef ref;
		ref.patches = { stod(p56), stod(p36), stod(p12), stod(p3) };
		gRadioRefs[band] = ref;
	}

	file.close();
	gLog << "  Loaded " << gRadioRefs.size() << " radiometric reference entries from: " << path << endl;
	return true;
}

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
				dns_b.push_back(avg[0]);
				dns_g.push_back(avg[1]);
				dns_r.push_back(avg[2]);
			} else {
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

RadioCoeffs getRadiometricCoeffs(const Mat& img, const string& filename, Point interval, int autoDetectThickness,
								  const string& radioDir, const string& templatePath) {
	RadioState state;
	Mat display;
	RadioCoeffs coeffs;

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
				Point p56_rel, p3_rel;
				double T = autoDetectThickness;
				double IW = bestSize.width - 2 * T;
				double IH = bestSize.height - 2 * T;

				if (bestRot == 0) {
					p56_rel = Point(bestSize.width / 2, T + IH / 8);
					p3_rel = Point(bestSize.width / 2, T + IH * 7 / 8);
				} else if (bestRot == 1) {
					p56_rel = Point(T + IW * 7 / 8, bestSize.height / 2);
					p3_rel = Point(T + IW / 8, bestSize.height / 2);
				} else if (bestRot == 2) {
					p56_rel = Point(bestSize.width / 2, T + IH * 7 / 8);
					p3_rel = Point(bestSize.width / 2, T + IH / 8);
				} else if (bestRot == 3) {
					p56_rel = Point(T + IW / 8, bestSize.height / 2);
					p3_rel = Point(T + IW * 7 / 8, bestSize.height / 2);
				}

				state.p56 = maxLoc_total + p56_rel;
				state.p3 = maxLoc_total + p3_rel;
				state.clicks = 2;
			}
		}
	}

	string winName = "Radiometric Calibration - " + filename;
	if (state.clicks < 2) {
		namedWindow(winName, WINDOW_NORMAL);
		setMouseCallback(winName, onMouseRadio, &state);
		cout << "Radiometric Calibration for " << filename << ":" << endl;
		cout << "  1. Click on the center of the 56% patch (usually the brightest/leftmost)." << endl;
		cout << "  2. Click on the center of the 3% patch (usually the darkest/rightmost)." << endl;
	}

	int boxSize = 5;
	while (state.clicks < 2) {
		Mat temp = display.clone();
		if (state.clicks >= 1) circle(temp, state.p56, boxSize, Scalar(0, 0, 255), -1);
		imshow(winName, temp);
		int key = waitKey(10);
		if (key == 27) {
			destroyWindow(winName);
			return coeffs;
		}
	}

	coeffs.filename = filename;
	string stem = path(filename).stem().string();
	vector<double> defaultTargets = {0.5647, 0.3582, 0.1148, 0.0272};
	vector<double> targets = defaultTargets;

	if (!stem.empty()) {
		char lastChar = stem.back();
		if (lastChar == '5') {
			auto it = gRadioRefs.find("5");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "NIR";
		} else if (lastChar == '4') {
			auto it = gRadioRefs.find("4");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "RedEdge";
		} else if (lastChar == '3') {
			auto it = gRadioRefs.find("3");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "Red";
		} else if (lastChar == '2') {
			auto it = gRadioRefs.find("2");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "Green";
		} else if (lastChar == '1') {
			auto it = gRadioRefs.find("1");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "Blue";
		} else if (lastChar == '0') {
			coeffs.isRGB = true;
			coeffs.bandName = "RGB";
			auto itR = gRadioRefs.find("0R"); auto itG = gRadioRefs.find("0G"); auto itB = gRadioRefs.find("0B");
			coeffs.targets_r = (itR != gRadioRefs.end()) ? itR->second.patches : defaultTargets;
			coeffs.targets_g = (itG != gRadioRefs.end()) ? itG->second.patches : defaultTargets;
			coeffs.targets_b = (itB != gRadioRefs.end()) ? itB->second.patches : defaultTargets;
		}
	}

	if (!coeffs.isRGB) coeffs.targets = targets;

	double stepX = (state.p3.x - state.p56.x) / 3.0;
	double stepY = (state.p3.y - state.p56.y) / 3.0;
	vector<double> dns_r, dns_g, dns_b;

	for (int i = 0; i < 4; ++i) {
		int cx = cvRound(state.p56.x + i * stepX);
		int cy = cvRound(state.p56.y + i * stepY);
		Rect roi(cx - boxSize / 2, cy - boxSize / 2, boxSize, boxSize);
		roi &= Rect(0, 0, img.cols, img.rows);

		if (roi.area() > 0) {
			Scalar avg = mean(img(roi));
			if (coeffs.isRGB && img.channels() >= 3) {
				dns_b.push_back(avg[0]); dns_g.push_back(avg[1]); dns_r.push_back(avg[2]);
			} else {
				dns_r.push_back(avg[0]);
			}
			rectangle(display, roi, Scalar(0, 255, 0), 1);
		} else {
			dns_r.push_back(0);
			if (coeffs.isRGB) { dns_g.push_back(0); dns_b.push_back(0); }
		}
	}

	if (coeffs.isRGB) { coeffs.dns_r = dns_r; coeffs.dns_g = dns_g; coeffs.dns_b = dns_b; }
	else { coeffs.dns = dns_r; }

	if (autoDetectThickness >= 0) {
		if (!exists(radioDir)) create_directories(radioDir);
		imwrite(radioDir + "/" + filename + "_board_preview.jpg", display);
	}

	auto solveCoeffs = [](const vector<double>& dns, const vector<double>& tgts) -> pair<double, double> {
		Mat X(4, 2, CV_64F); Mat Y(4, 1, CV_64F);
		for (int i = 0; i < 4; ++i) { X.at<double>(i, 0) = dns[i]; X.at<double>(i, 1) = 1.0; Y.at<double>(i, 0) = tgts[i]; }
		Mat sol; solve(X, Y, sol, DECOMP_SVD);
		return {sol.at<double>(0, 0), sol.at<double>(1, 0)};
	};

	if (coeffs.isRGB && img.channels() >= 3) {
		auto [a_r, b_r] = solveCoeffs(dns_r, coeffs.targets_r);
		auto [a_g, b_g] = solveCoeffs(dns_g, coeffs.targets_g);
		auto [a_b, b_b] = solveCoeffs(dns_b, coeffs.targets_b);
		coeffs.a_r = a_r; coeffs.b_r = b_r; coeffs.a_g = a_g; coeffs.b_g = b_g; coeffs.a_b = a_b; coeffs.b_b = b_b;
		coeffs.valid = true;
	} else {
		auto [a, b] = solveCoeffs(dns_r, targets);
		coeffs.a = a; coeffs.b = b; coeffs.valid = true;
	}

	coeffs.p56 = state.p56; coeffs.p3 = state.p3; coeffs.boxSize = boxSize;
	return coeffs;
}

RadioCoeffs getRadiometricCoeffsByEdge(const Mat& img, const string& filename, Point interval, int autoDetectThickness,
								  const string& radioDir, const string& templatePath) {
	RadioState state;
	Mat display;
	RadioCoeffs coeffs;

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
			cout << "  Auto-detecting board using COMBINED (Intensity + Edge) matching..." << endl;
			double maxVal_total = -1;
			Point maxLoc_total;
			int bestRot = 0;
			double bestScale = 1.0;
			Size bestSize;

			// --- 1. Prepare Display Image (Intensity & Gradients) ---
			Mat grayDisplay;
			cvtColor(display, grayDisplay, COLOR_BGR2GRAY);
			
			Mat gradX_disp, gradY_disp, magDisplay;
			Sobel(grayDisplay, gradX_disp, CV_32F, 1, 0, 3);
			Sobel(grayDisplay, gradY_disp, CV_32F, 0, 1, 3);
			magnitude(gradX_disp, gradY_disp, magDisplay);
			normalize(magDisplay, magDisplay, 0, 255, NORM_MINMAX, CV_8U); 

			// --- 2. Prepare Template (Intensity & Gradients) ---
			Mat baseGrayTempl;
			cvtColor(templ, baseGrayTempl, COLOR_BGR2GRAY);
			
			Mat gradX_t, gradY_t, magBaseTempl;
			Sobel(baseGrayTempl, gradX_t, CV_32F, 1, 0, 3);
			Sobel(baseGrayTempl, gradY_t, CV_32F, 0, 1, 3);
			magnitude(gradX_t, gradY_t, magBaseTempl);
			normalize(magBaseTempl, magBaseTempl, 0, 255, NORM_MINMAX, CV_8U);

			// --- 3. Broader Scale Search ---
			vector<double> scales;
			for(double s = 0.2; s <= 2.0; s += 0.15) { 
				scales.push_back(s);
			}

			for (double scale : scales) {
				Mat scaledMagTempl, scaledGrayTempl;
				resize(magBaseTempl, scaledMagTempl, Size(), scale, scale);
                resize(baseGrayTempl, scaledGrayTempl, Size(), scale, scale);

				for (int rot = 0; rot < 4; ++rot) {
					Mat rotMagTempl, rotGrayTempl;
					if (rot == 0) { rotMagTempl = scaledMagTempl; rotGrayTempl = scaledGrayTempl; }
					else if (rot == 1) { rotate(scaledMagTempl, rotMagTempl, ROTATE_90_CLOCKWISE); rotate(scaledGrayTempl, rotGrayTempl, ROTATE_90_CLOCKWISE); }
					else if (rot == 2) { rotate(scaledMagTempl, rotMagTempl, ROTATE_180); rotate(scaledGrayTempl, rotGrayTempl, ROTATE_180); }
					else if (rot == 3) { rotate(scaledMagTempl, rotMagTempl, ROTATE_90_COUNTERCLOCKWISE); rotate(scaledGrayTempl, rotGrayTempl, ROTATE_90_COUNTERCLOCKWISE); }

					if (rotMagTempl.cols > magDisplay.cols || rotMagTempl.rows > magDisplay.rows) continue;

					// A. Match Intensity
					Mat res_intensity;
					matchTemplate(grayDisplay, rotGrayTempl, res_intensity, TM_CCOEFF_NORMED);
                    // Remove negative correlations (inversions) to prevent them from breaking the math
                    threshold(res_intensity, res_intensity, 0, 1.0, THRESH_TOZERO); 

                    // B. Match Edges
                    Mat res_edges;
					matchTemplate(magDisplay, rotMagTempl, res_edges, TM_CCOEFF_NORMED);
                    threshold(res_edges, res_edges, 0, 1.0, THRESH_TOZERO);

                    // C. Combine Maps (50% Intensity, 50% Edges)
                    // The true board will peak heavily in BOTH maps at the exact same pixel.
                    Mat result_combined;
                    addWeighted(res_intensity, 0.5, res_edges, 0.5, 0.0, result_combined);
					
					double minVal, maxVal;
					Point minLoc, maxLoc;
					minMaxLoc(result_combined, &minVal, &maxVal, &minLoc, &maxLoc);

					if (maxVal > maxVal_total) {
						maxVal_total = maxVal;
						maxLoc_total = maxLoc;
						bestRot = rot;
						bestScale = scale;
						bestSize = rotMagTempl.size();
					}
				}
			}

			// Threshold is slightly lower because combined maps rarely hit perfect 1.0
			if (maxVal_total > 0.45) { 
				cout << "    Board detected! (score=" << maxVal_total << ", rot=" << bestRot*90 << "deg, scale=" << bestScale << ")" << endl;
				Point p56_rel, p3_rel;
                
                // CRITICAL FIX: The thickness of the border scales with the template!
				double T = autoDetectThickness * bestScale; 
				double IW = bestSize.width - 2 * T;
				double IH = bestSize.height - 2 * T;

                // NOTE: This logic assumes a 4-patch board. If your template is a 3-patch board, 
                // the /8 and *7/8 math below will not land on the correct patches.
				if (bestRot == 0) {
					p56_rel = Point(bestSize.width / 2, T + IH / 8);
					p3_rel = Point(bestSize.width / 2, T + IH * 7 / 8);
				} else if (bestRot == 1) {
					p56_rel = Point(T + IW * 7 / 8, bestSize.height / 2);
					p3_rel = Point(T + IW / 8, bestSize.height / 2);
				} else if (bestRot == 2) {
					p56_rel = Point(bestSize.width / 2, T + IH * 7 / 8);
					p3_rel = Point(bestSize.width / 2, T + IH / 8);
				} else if (bestRot == 3) {
					p56_rel = Point(T + IW / 8, bestSize.height / 2);
					p3_rel = Point(T + IW * 7 / 8, bestSize.height / 2);
				}

				state.p56 = maxLoc_total + p56_rel;
				state.p3 = maxLoc_total + p3_rel;
				state.clicks = 2;
			} else {
                cout << "    Failed to detect board (highest score: " << maxVal_total << ")" << endl;
            }
		} else {
            cout << "    Warning: Could not load template file!" << endl;
        }
	}

	string winName = "Radiometric Calibration - " + filename;
	if (state.clicks < 2) {
		namedWindow(winName, WINDOW_NORMAL);
		setMouseCallback(winName, onMouseRadio, &state);
		cout << "Radiometric Calibration for " << filename << ":" << endl;
		cout << "  1. Click on the center of the 56% patch (usually the brightest/leftmost)." << endl;
		cout << "  2. Click on the center of the 3% patch (usually the darkest/rightmost)." << endl;
	}

	int boxSize = 5;
	while (state.clicks < 2) {
		Mat temp = display.clone();
		if (state.clicks >= 1) circle(temp, state.p56, boxSize, Scalar(0, 0, 255), -1);
		imshow(winName, temp);
		int key = waitKey(10);
		if (key == 27) {
			destroyWindow(winName);
			return coeffs;
		}
	}

	coeffs.filename = filename;
	string stem = path(filename).stem().string();
	vector<double> defaultTargets = {0.5647, 0.3582, 0.1148, 0.0272};
	vector<double> targets = defaultTargets;

	if (!stem.empty()) {
		char lastChar = stem.back();
		if (lastChar == '5') {
			auto it = gRadioRefs.find("5");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "NIR";
		} else if (lastChar == '4') {
			auto it = gRadioRefs.find("4");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "RedEdge";
		} else if (lastChar == '3') {
			auto it = gRadioRefs.find("3");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "Red";
		} else if (lastChar == '2') {
			auto it = gRadioRefs.find("2");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "Green";
		} else if (lastChar == '1') {
			auto it = gRadioRefs.find("1");
			targets = (it != gRadioRefs.end()) ? it->second.patches : defaultTargets;
			coeffs.bandName = "Blue";
		} else if (lastChar == '0') {
			coeffs.isRGB = true;
			coeffs.bandName = "RGB";
			auto itR = gRadioRefs.find("0R"); auto itG = gRadioRefs.find("0G"); auto itB = gRadioRefs.find("0B");
			coeffs.targets_r = (itR != gRadioRefs.end()) ? itR->second.patches : defaultTargets;
			coeffs.targets_g = (itG != gRadioRefs.end()) ? itG->second.patches : defaultTargets;
			coeffs.targets_b = (itB != gRadioRefs.end()) ? itB->second.patches : defaultTargets;
		}
	}

	if (!coeffs.isRGB) coeffs.targets = targets;

	double stepX = (state.p3.x - state.p56.x) / 3.0;
	double stepY = (state.p3.y - state.p56.y) / 3.0;
	vector<double> dns_r, dns_g, dns_b;

	for (int i = 0; i < 4; ++i) {
		int cx = cvRound(state.p56.x + i * stepX);
		int cy = cvRound(state.p56.y + i * stepY);
		Rect roi(cx - boxSize / 2, cy - boxSize / 2, boxSize, boxSize);
		roi &= Rect(0, 0, img.cols, img.rows);

		if (roi.area() > 0) {
			Scalar avg = mean(img(roi));
			if (coeffs.isRGB && img.channels() >= 3) {
				dns_b.push_back(avg[0]); dns_g.push_back(avg[1]); dns_r.push_back(avg[2]);
			} else {
				dns_r.push_back(avg[0]);
			}
			rectangle(display, roi, Scalar(0, 255, 0), 1);
		} else {
			dns_r.push_back(0);
			if (coeffs.isRGB) { dns_g.push_back(0); dns_b.push_back(0); }
		}
	}

	if (coeffs.isRGB) { coeffs.dns_r = dns_r; coeffs.dns_g = dns_g; coeffs.dns_b = dns_b; }
	else { coeffs.dns = dns_r; }

	if (autoDetectThickness >= 0) {
		if (!exists(radioDir)) create_directories(radioDir);
		imwrite(radioDir + "/" + filename + "_board_preview.jpg", display);
	}

	auto solveCoeffs = [](const vector<double>& dns, const vector<double>& tgts) -> pair<double, double> {
		Mat X(4, 2, CV_64F); Mat Y(4, 1, CV_64F);
		for (int i = 0; i < 4; ++i) { X.at<double>(i, 0) = dns[i]; X.at<double>(i, 1) = 1.0; Y.at<double>(i, 0) = tgts[i]; }
		Mat sol; solve(X, Y, sol, DECOMP_SVD);
		return {sol.at<double>(0, 0), sol.at<double>(1, 0)};
	};

	if (coeffs.isRGB && img.channels() >= 3) {
		auto [a_r, b_r] = solveCoeffs(dns_r, coeffs.targets_r);
		auto [a_g, b_g] = solveCoeffs(dns_g, coeffs.targets_g);
		auto [a_b, b_b] = solveCoeffs(dns_b, coeffs.targets_b);
		coeffs.a_r = a_r; coeffs.b_r = b_r; coeffs.a_g = a_g; coeffs.b_g = b_g; coeffs.a_b = a_b; coeffs.b_b = b_b;
		coeffs.valid = true;
	} else {
		auto [a, b] = solveCoeffs(dns_r, targets);
		coeffs.a = a; coeffs.b = b; coeffs.valid = true;
	}

	coeffs.p56 = state.p56; coeffs.p3 = state.p3; coeffs.boxSize = boxSize;
	return coeffs;
}

Mat applyRadiometricCalibration(const Mat& img, RadioCoeffs coeffs) {
	if (!coeffs.valid) return img.clone();
	Mat calibrated;
	if (coeffs.isRGB && img.channels() >= 3) {
		vector<Mat> channels(3); split(img, channels);
		channels[0].convertTo(channels[0], CV_64F, coeffs.a_b, coeffs.b_b);
		channels[1].convertTo(channels[1], CV_64F, coeffs.a_g, coeffs.b_g);
		channels[2].convertTo(channels[2], CV_64F, coeffs.a_r, coeffs.b_r);
		merge(channels, calibrated);
	} else {
		img.convertTo(calibrated, CV_64F, coeffs.a, coeffs.b);
	}
	double minR, maxR; minMaxLoc(calibrated, &minR, &maxR);
	if (maxR == minR) maxR = minR + 1.0;
	Mat normalized;
	calibrated.convertTo(normalized, CV_8U, 255.0 / (maxR - minR), -minR * 255.0 / (maxR - minR));
	return normalized;
}

void exportRadiometricCsv(const string& outPath, const map<string, GroupData>& allGroups) {
	string csvPath = outPath + "/radiometric_report.csv";
	ofstream csv(csvPath);
	if (!csv.is_open()) return;
	csv << fixed << setprecision(6) << "Filename,";
	for (int band = 1; band <= 5; ++band) csv << "DN_B" << band << "_P1,DN_B" << band << "_P2,DN_B" << band << "_P3,DN_B" << band << "_P4,";
	csv << "DN_RGB_R_P1,DN_RGB_R_P2,DN_RGB_R_P3,DN_RGB_R_P4,DN_RGB_G_P1,DN_RGB_G_P2,DN_RGB_G_P3,DN_RGB_G_P4,DN_RGB_B_P1,DN_RGB_B_P2,DN_RGB_B_P3,DN_RGB_B_P4,Slope (a),Intercept (b)" << endl;
	for (const auto& [prefix, data] : allGroups) {
		if (!data.coeffs.valid) continue;
		csv << data.coeffs.filename << ",";
		for (int band = 1; band <= 5; ++band) {
			auto it = data.multispectralDns.find(band);
			if (it != data.multispectralDns.end() && it->second.size() == 4) csv << it->second[0] << "," << it->second[1] << "," << it->second[2] << "," << it->second[3] << ",";
			else csv << "0,0,0,0,";
		}
		if (data.rgbCollected) {
			csv << data.rgbDns_r[0] << "," << data.rgbDns_r[1] << "," << data.rgbDns_r[2] << "," << data.rgbDns_r[3] << ",";
			csv << data.rgbDns_g[0] << "," << data.rgbDns_g[1] << "," << data.rgbDns_g[2] << "," << data.rgbDns_g[3] << ",";
			csv << data.rgbDns_b[0] << "," << data.rgbDns_b[1] << "," << data.rgbDns_b[2] << "," << data.rgbDns_b[3] << ",";
		} else {
			csv << "0,0,0,0,0,0,0,0,0,0,0,0,";
		}
		if (data.coeffs.isRGB) csv << (data.coeffs.a_r + data.coeffs.a_g + data.coeffs.a_b) / 3.0 << "," << (data.coeffs.b_r + data.coeffs.b_g + data.coeffs.b_b) / 3.0 << endl;
		else csv << data.coeffs.a << "," << data.coeffs.b << endl;
	}
	csv.close();
}

// --- VEGETATION INDICES ---

Mat calculateVegIndex(const string& type, const map<int, Mat>& bands, const Mat& mask) {
	// https://github.com/px39n/Awesome-Vegetation-Index
	
	Mat nir, red, green, blue, re, swir1, swir2;
	if (bands.count(5)) bands.at(5).convertTo(nir, CV_32F);
	else if (bands.count(4)) bands.at(4).convertTo(nir, CV_32F);
	if (bands.count(3)) bands.at(3).convertTo(red, CV_32F);
	else if (bands.count(2) && !bands.count(3)) bands.at(2).convertTo(red, CV_32F);
	if (bands.count(2)) bands.at(2).convertTo(green, CV_32F);
	else if (bands.count(1) && !bands.count(2)) bands.at(1).convertTo(green, CV_32F);
	if (bands.count(1)) bands.at(1).convertTo(blue, CV_32F);
	if (bands.count(4)) bands.at(4).convertTo(re, CV_32F);
	else if (bands.count(3) && !bands.count(4)) bands.at(3).convertTo(re, CV_32F);
	if (bands.count(6)) bands.at(6).convertTo(swir1, CV_32F);
	if (bands.count(7)) bands.at(7).convertTo(swir2, CV_32F);

	// Apply mask before calculation if provided
	if (!mask.empty()) {
		if (!nir.empty()) nir.setTo(0, mask == 0);
		if (!red.empty()) red.setTo(0, mask == 0);
		if (!green.empty()) green.setTo(0, mask == 0);
		if (!blue.empty()) blue.setTo(0, mask == 0);
		if (!re.empty()) re.setTo(0, mask == 0);
		if (!swir1.empty()) swir1.setTo(0, mask == 0);
		if (!swir2.empty()) swir2.setTo(0, mask == 0);
	}

	Mat result;
	const float eps = 1e-6f;

	if (type == "ndvi") { 
		if (nir.empty() || red.empty()) return Mat(); 
		result = (nir - red) / (nir + red + eps); 
	}
	else if (type == "sr" || type == "rvi") { 
		if (nir.empty() || red.empty()) return Mat(); 
		result = nir / (red + eps); 
	}
	else if (type == "rsr") {
		if (nir.empty() || red.empty() || swir1.empty()) return Mat();
		double minV, maxV; minMaxLoc(swir1, &minV, &maxV);
		result = (nir / (red + eps)) * (1.0f - (swir1 - (float)minV) / ((float)maxV - (float)minV + eps));
	}
	else if (type == "arvi") {
		if (nir.empty() || red.empty() || blue.empty()) return Mat();
		Mat rb = red - 1.0f * (blue - red);
		result = (nir - rb) / (nir + rb + eps);
	}
	else if (type == "afri1.6") {
		if (nir.empty() || swir1.empty()) return Mat();
		result = (nir - 0.66f * swir1) / (nir + 0.66f * swir1 + eps);
	}
	else if (type == "afri2.1") {
		if (nir.empty() || swir2.empty()) return Mat();
		result = (nir - 0.5f * swir2) / (nir + 0.5f * swir2 + eps);
	}
	else if (type == "vari") {
		if (green.empty() || red.empty() || blue.empty()) return Mat();
		result = (green - red) / (green + red - blue + eps);
	}
	else if (type == "tvi") {
		if (nir.empty() || green.empty() || red.empty()) return Mat();
		result = 0.5f * (120.0f * (nir - green) - 200.0f * (red - green));
	}
	else if (type == "msarvi") {
		if (nir.empty() || red.empty() || blue.empty()) return Mat();
		Mat rb = red - 1.0f * (blue - red);
		Mat term = (2.0f * nir + 1.0f);
		Mat sq; pow(term, 2, sq);
		Mat root; cv::sqrt(sq - 8.0f * (nir - rb), root);
		result = (term - root) / 2.0f;
	}
	else if (type == "gemi") {
		if (nir.empty() || red.empty()) return Mat();
		Mat eta = (2.0f * (nir.mul(nir) - red.mul(red)) + 1.5f * nir + 0.5f * red) / (nir + red + 0.5f + eps);
		result = eta.mul(1.0f - 0.25f * eta) - ((red - 0.125f) / (1.0f - red + eps));
	}
	else if (type == "evi") { 
		if (nir.empty() || red.empty() || blue.empty()) return Mat(); 
		result = 2.5f * (nir - red) / (nir + 6.0f * red - 7.5f * blue + 1.0f); 
	}
	else if (type == "evi2") {
		if (nir.empty() || red.empty()) return Mat();
		result = 2.5f * (nir - red) / (nir + 2.4f * red + 1.0f);
	}
	else if (type == "mcari") {
		if (re.empty() || red.empty() || green.empty()) return Mat();
		result = ((re - red) - 0.2f * (re - green)).mul(re / (red + eps));
	}
	else if (type == "osavi") { 
		if (nir.empty() || red.empty()) return Mat(); 
		result = 1.16f * (nir - red) / (nir + red + 0.16f); 
	}
	else if (type == "savi") {
		if (nir.empty() || red.empty()) return Mat();
		float L = 0.5f;
		result = (1.0f + L) * (nir - red) / (nir + red + L);
	}
	else if (type == "msavi") {
		if (nir.empty() || red.empty()) return Mat();
		Mat term = (2.0f * nir + 1.0f);
		Mat sq; pow(term, 2, sq);
		Mat root; cv::sqrt(sq - 8.0f * (nir - red), root);
		result = (term - root) / 2.0f;
	}
	else if (type == "dvi") {
		if (nir.empty() || red.empty()) return Mat();
		result = nir - red;
	}
	else if (type == "nri") {
		if (red.empty() || green.empty()) return Mat();
		result = (red - green) / (red + green + eps);
	}
	else if (type == "ndre") { 
		if (nir.empty() || re.empty()) return Mat(); 
		result = (nir - re) / (nir + re + eps); 
	}
	else if (type == "gndvi") { 
		if (nir.empty() || green.empty()) return Mat(); 
		result = (nir - green) / (nir + green + eps); 
	}
	else if (type == "cigreen") {
		if (nir.empty() || green.empty()) return Mat();
		result = (nir / (green + eps)) - 1.0f;
	}
	else if (type == "cire") {
		if (nir.empty() || re.empty()) return Mat();
		result = (nir / (re + eps)) - 1.0f;
	}
	else if (type == "ipvi") {
		if (nir.empty() || red.empty()) return Mat();
		result = nir / (nir + red + eps);
	}
	else if (type == "rdvi") { 
		if (nir.empty() || red.empty()) return Mat(); 
		Mat sum_val; cv::sqrt(nir + red, sum_val); 
		result = (nir - red) / (sum_val + eps); 
	}
	else if (type == "wdrvi") {
		if (nir.empty() || red.empty()) return Mat();
		float alpha = 0.1f;
		result = (alpha * nir - red) / (alpha * nir + red + eps);
	}
	else if (type == "wdvi") {
		if (nir.empty() || red.empty()) return Mat();
		float a = 1.0f; // default soil line slope
		result = nir - a * red;
	}
	else if (type == "tsavi") {
		if (nir.empty() || red.empty()) return Mat();
		float a = 1.0f, b = 0.0f, X = 0.08f;
		result = (a * (nir - a * red - b)) / (red + a * nir - a * b + X * (1.0f + a * a) + eps);
	}
	else if (type == "atsavi") {
		if (nir.empty() || red.empty()) return Mat();
		float a = 1.0f, b = 0.0f, X = 0.08f;
		result = (a * (nir - a * red - b)) / (a * nir + red - a * b + X * (1.0f + a * a) + eps);
	}
	else if (type == "msr") { 
		if (nir.empty() || red.empty()) return Mat(); 
		Mat ratio = nir / (red + eps); 
		Mat sqrt_ratio; cv::sqrt(ratio, sqrt_ratio); 
		result = (ratio - 1.0f) / (sqrt_ratio + 1.0f + eps); 
	}

	if (!result.empty()) { 
		threshold(result, result, 0, 0, THRESH_TOZERO); 
		threshold(result, result, 1, 1, THRESH_TRUNC); 
	}
	return result;
}

// ============================================================================
// HELPER 1: Background Subtraction (Concrete vs. Non-Concrete)
// ============================================================================
static Mat computeNonConcreteMask(const Mat& rgbImg) {
	// Concrete floor is grey: Low Saturation (S in HSV) and Low Color Variance (|R-G|, |G-B|)
	Mat hsv, nonConcreteMask;
	cvtColor(rgbImg, hsv, COLOR_BGR2HSV);

	vector<Mat> hsvChannels;
	split(hsv, hsvChannels);
	Mat saturation = hsvChannels[1]; // S channel

	// 1. Saturation threshold: Concrete S is typically < 20 (scale 0-255)
	Mat highSatMask = saturation > 22;

	// 2. Channel Delta Check: max(R,G,B) - min(R,G,B)
	vector<Mat> bgr;
	split(rgbImg, bgr);
	Mat maxBGR, minBGR, chroma;
	max(bgr[0], max(bgr[1], bgr[2]), maxBGR);
	min(bgr[0], min(bgr[1], bgr[2]), minBGR);
	chroma = maxBGR - minBGR;

	Mat chromaMask = chroma > 12; // True if there is actual color hue

	// Combine: A pixel is non-concrete if it has sufficient saturation OR chroma
	bitwise_or(highSatMask, chromaMask, nonConcreteMask);
	return nonConcreteMask;
}

// ============================================================================
// GENTLE HELPER 1: Soft Background Rejection (Preserves Yellow/Brown Leaves)
// ============================================================================
Mat computeSoftNonConcreteMask(const Mat& rgbImg) {
	Mat hsv;
	cvtColor(rgbImg, hsv, COLOR_BGR2HSV);

	vector<Mat> hsvChannels;
	split(hsv, hsvChannels);
	Mat saturation = hsvChannels[1]; // S channel (0-255)

	// Max channel difference: max(R,G,B) - min(R,G,B)
	vector<Mat> bgr;
	split(rgbImg, bgr);
	Mat maxBGR, minBGR, chroma;
	max(bgr[0], max(bgr[1], bgr[2]), maxBGR);
	min(bgr[0], min(bgr[1], bgr[2]), minBGR);
	chroma = maxBGR - minBGR;

	// A pixel is NOT concrete if it has ANY color variation (chroma > 8)
	// OR minimal saturation (S > 12). Retains pale yellow / brown leaves.
	Mat nonConcreteMask;
	Mat lowChromaMask = chroma > 8;
	Mat lowSatMask = saturation > 12;
	bitwise_or(lowChromaMask, lowSatMask, nonConcreteMask);
	return nonConcreteMask;
}

// ============================================================================
// GENTLE FIX 2: Central Anchor Enclosure (Preserves Sparse & Small Distant Leaves)
// ============================================================================
Mat applyCentralAnchorEnclosure(const Mat& candidateMask, double maxRadiusRatio) {
	vector<Point> points;
	findNonZero(candidateMask, points);

	if (points.empty()) return candidateMask.clone();

	// Median of candidate pixel coordinates: robust against outer moss outliers
	vector<int> xCoords, yCoords;
	xCoords.reserve(points.size());
	yCoords.reserve(points.size());
	for (const auto& p : points) {
		xCoords.push_back(p.x);
		yCoords.push_back(p.y);
	}

	size_t n = xCoords.size() / 2;
	nth_element(xCoords.begin(), xCoords.begin() + n, xCoords.end());
	nth_element(yCoords.begin(), yCoords.begin() + n, yCoords.end());

	Point2f centralAnchor((float)xCoords[n], (float)yCoords[n]);

	// Maximum allowed radius from the central plant anchor
	double maxRadiusPx = candidateMask.cols * maxRadiusRatio;

	// Keep ANY contour whose centroid lies within the Central Anchor Enclosure
	vector<vector<Point>> contours;
	findContours(candidateMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	Mat outputMask = Mat::zeros(candidateMask.size(), CV_8U);
	for (size_t i = 0; i < contours.size(); ++i) {
		double area = contourArea(contours[i]);

		// Lower minimum area limit to retain small unhealthy leaves
		if (area >= 5.0) {
			Moments mu = moments(contours[i]);
			Point2f contourCentroid(mu.m10 / (mu.m00 + 1e-6), mu.m01 / (mu.m00 + 1e-6));

			double distToAnchor = cv::norm(contourCentroid - centralAnchor);

			// If the leaf is within the plant's spatial growth zone, KEEP IT
			if (distToAnchor <= maxRadiusPx) {
				drawContours(outputMask, contours, (int)i, Scalar(255), FILLED);
			}
		}
	}

	return outputMask;
}

// ============================================================================
// HELPER 2: Texture / High-Frequency Variance Filtering
// ============================================================================
static Mat computeSmoothTextureMask(const Mat& rgbImg, double maxTextureThresh = 32.0) {
	Mat gray, lap, absLap, localTexture;
	cvtColor(rgbImg, gray, COLOR_BGR2GRAY);

	// Laplacian calculates spatial intensity gradients
	Laplacian(gray, lap, CV_32F, 3);
	convertScaleAbs(lap, absLap);

	// Blur to compute local gradient density (moss/concrete = high gradient variance)
	blur(absLap, localTexture, Size(7, 7));

	Mat smoothMask;
	// Keep pixels with low local gradient variance (smooth leaf surfaces)
	threshold(localTexture, smoothMask, maxTextureThresh, 255, THRESH_BINARY_INV);
	return smoothMask;
}

// ============================================================================
// HELPER 3: Low-NDVI Thresholding (Supports Yellow / Stressed Leaves)
// ============================================================================
static Mat computeLowNDVIMask(const map<int, Mat>& bands, Size targetSize, float lowNdviThresh = 0.12f) {
	const float eps = 1e-6f;
	Mat ndviMask;

	// Band 5 = NIR, Band 3 = Red (or Band 2 Green fallback)
	if (bands.count(5) && (bands.count(3) || bands.count(2))) {
		Mat NIR = bands.at(5).clone();
		Mat red = bands.count(3) ? bands.at(3).clone() : bands.at(2).clone();

		if (NIR.channels() == 1 && red.channels() == 1) {
			resize(NIR, NIR, targetSize);
			resize(red, red, targetSize);

			Mat NIRf, redf;
			NIR.convertTo(NIRf, CV_32F);
			red.convertTo(redf, CV_32F);

			// Compute NDVI = (NIR - Red) / (NIR + Red)
			Mat NDVI = (NIRf - redf) / (NIRf + redf + eps);

			// Low threshold (0.12 - 0.15): Captures yellow/stressed leaves while excluding concrete (~0.02)
			Mat ndvi8;
			threshold(NDVI, ndvi8, lowNdviThresh, 255, THRESH_BINARY);
			ndvi8.convertTo(ndviMask, CV_8U);
			return ndviMask;
		}
	}

	// Return empty if multispectral bands are missing
	return Mat();
}

Mat refineMaskWithKMeans(const Mat& candidateMask, const Mat& rgbImg, const Mat& ndviImg) {
	const double eps = 1e-6;
	vector<Point> candPoints;
	findNonZero(candidateMask, candPoints);

	// Too few pixels to cluster meaningfully
	if ((int)candPoints.size() < 100) return candidateMask.clone();

	Mat hsv;
	cvtColor(rgbImg, hsv, COLOR_BGR2HSV);

	// Feature matrix: N rows x 3 cols [Normalized_Hue, Saturation, NDVI]
	Mat samples((int)candPoints.size(), 3, CV_32F);
	for (size_t i = 0; i < candPoints.size(); ++i) {
		const Point& pt = candPoints[i];
		Vec3b hsvPixel = hsv.at<Vec3b>(pt);
		double ndviVal = 0.0;
		if (!ndviImg.empty() && ndviImg.type() == CV_32F) ndviVal = ndviImg.at<float>(pt);

		samples.at<float>((int)i, 0) = hsvPixel[0] / 180.0f;          // Hue
		samples.at<float>((int)i, 1) = hsvPixel[1] / 255.0f;          // Saturation
		samples.at<float>((int)i, 2) = (float)std::max(0.0, ndviVal); // NDVI
	}

	// K-Means with K = 2
	const int K = 2;
	Mat labels, centers;
	TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0);
	kmeans(samples, K, labels, criteria, 3, KMEANS_PP_CENTERS, centers);

	// Plant cluster = higher combined (Saturation + NDVI) score
	const float score0 = centers.at<float>(0, 1) + centers.at<float>(0, 2);
	const float score1 = centers.at<float>(1, 1) + centers.at<float>(1, 2);
	const int plantClusterIdx = (score1 > score0) ? 1 : 0;
	const int mossClusterIdx = 1 - plantClusterIdx;

	// Conservatively keep the whole mask unless the "moss" cluster is CLEARLY
	// inferior. If the two clusters are close in (Saturation+NDVI) they are both
	// legit canopy foliage and pruning would amputate healthy leaves.
	const float scorePlant = std::max(score0, score1);
	const float scoreMoss = std::min(score0, score1);
	// Require a sizeable spectral gap so we never split one plant into two halves.
	if (scoreMoss > 0.85f * scorePlant) {
		return candidateMask.clone();
	}

	// Drop only the pixels belonging to the clearly-inferior spectral cluster.
	Mat refinedMask = candidateMask.clone();
	for (size_t i = 0; i < candPoints.size(); ++i) {
		if (labels.at<int>((int)i) == mossClusterIdx) {
			refinedMask.at<uchar>(candPoints[i].y, candPoints[i].x) = 0;
		}
	}
	return refinedMask;
}

Mat filterSatelliteClusters(const Mat& inputMask, double maxMergeDistancePx) {
	const double eps = 1e-6;
	vector<vector<Point>> contours;
	findContours(inputMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	if (contours.size() <= 1) return inputMask.clone();

	const int n = (int)contours.size();
	vector<Point2f> centroids(n);
	vector<double> areas(n);

	for (int i = 0; i < n; ++i) {
		areas[i] = contourArea(contours[i]);
		Moments mu = moments(contours[i]);
		centroids[i] = Point2f(mu.m10 / (mu.m00 + eps), mu.m01 / (mu.m00 + eps));
	}

	// Adjacency grouping: merge contours whose centroids are within maxMergeDistancePx
	vector<int> clusterLabels(n, -1);
	int currentCluster = 0;
	vector<int> queue;
	for (int i = 0; i < n; ++i) {
		if (clusterLabels[i] != -1) continue;
		clusterLabels[i] = currentCluster;
		queue.clear();
		queue.push_back(i);
		while (!queue.empty()) {
			int curr = queue.back();
			queue.pop_back();
			for (int j = 0; j < n; ++j) {
				if (clusterLabels[j] == -1 && norm(centroids[curr] - centroids[j]) <= maxMergeDistancePx) {
					clusterLabels[j] = currentCluster;
					queue.push_back(j);
				}
			}
		}
		currentCluster++;
	}

	// Sum total area per cluster
	vector<double> clusterAreas(currentCluster, 0.0);
	for (int i = 0; i < n; ++i) clusterAreas[clusterLabels[i]] += areas[i];

	const int bestClusterIdx = (int)(max_element(clusterAreas.begin(), clusterAreas.end()) - clusterAreas.begin());
	const double primaryArea = clusterAreas[bestClusterIdx];

	// Reconstruct mask. Keep the primary cluster plus every cluster that is NOT a
	// tiny satellite (>= 5% of the primary's area). Only isolated specks get dropped,
	// so fragmented real canopy is preserved while moss/algae satellites are removed.
	Mat cleanMask = Mat::zeros(inputMask.size(), CV_8U);
	for (int i = 0; i < n; ++i) {
		if (clusterLabels[i] == bestClusterIdx || clusterAreas[clusterLabels[i]] >= 0.05 * primaryArea) {
			drawContours(cleanMask, contours, i, Scalar(255), FILLED);
		}
	}
	return cleanMask;
}

GreenMaskResults applyGreenMask(cv::Mat& indexImg, const cv::Mat& rgbImg, const string& outputDir, const string& prefix, const string& indexName, const map<int, Mat>& bands, const GreenMaskParams& params) {
	GreenMaskResults results;
	if (rgbImg.empty()) return results;

	// ==========================================================================
	// GENTLE MODE (Stressed-Plant Friendly): preserves yellow/brown/sparse leaves
	// ==========================================================================
	if (params.gentleMode) {
		// 1. Soft background subtraction (keeps yellow/brown: any chroma or saturation)
		Mat nonConcrete = computeSoftNonConcreteMask(rgbImg);

		// 2. Relaxed texture & low-NDVI thresholds
		Mat textMask = computeSmoothTextureMask(rgbImg, params.gentleTextThresh);
		Mat ndviMask = computeLowNDVIMask(bands, rgbImg.size(), (float)params.gentleNdviThresh);

		Mat candidateMask;
		if (!ndviMask.empty()) {
			bitwise_and(nonConcrete, ndviMask, candidateMask);
		} else {
			candidateMask = nonConcrete;
		}
		bitwise_and(candidateMask, textMask, candidateMask);

		// 3. Light morphology (3x3) so tiny leaves are not eroded away
		medianBlur(candidateMask, candidateMask, 3);
		Mat kernelOpen = getStructuringElement(MORPH_ELLIPSE, Size(params.gentleOpenKernel, params.gentleOpenKernel));
		morphologyEx(candidateMask, candidateMask, MORPH_OPEN, kernelOpen);

		// 4. Central Anchor Enclosure (replaces aggressive K-Means/Spatial clustering)
		Mat finalMask = applyCentralAnchorEnclosure(candidateMask, params.anchorRadiusRatio);

		// 5. Aggregate final statistics
		double totalValidArea = 0;
		Point2f weightedCentroid(0, 0);
		vector<Point> allPlantPoints;
		vector<vector<Point>> contours;
		findContours(finalMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (const auto& c : contours) {
			double a = contourArea(c);
			if (a >= params.gentleMinArea) {
				Moments mu = moments(c);
				Point2f ctd(mu.m10 / (mu.m00 + 1e-6), mu.m01 / (mu.m00 + 1e-6));
				totalValidArea += a;
				weightedCentroid += ctd * (float)a;
				for (const auto& p : c) allPlantPoints.push_back(p);
			}
		}

		if (totalValidArea > 0) {
			results.mask = finalMask;
			results.totalArea = totalValidArea;
			results.centroid = weightedCentroid / (float)(totalValidArea + 1e-6);
			results.valid = true;
			if (allPlantPoints.size() >= 5) {
				convexHull(allPlantPoints, results.convexHull);
				results.ellipse = fitEllipse(allPlantPoints);
			}
		} else {
			results.mask = Mat::zeros(rgbImg.size(), CV_8U);
			results.valid = false;
		}

		if (!indexImg.empty() && indexImg.size() == results.mask.size()) {
			indexImg.setTo(0, results.mask == 0);
		}
		return results;
	}

	// ------------------------------------------------------------------------
	// STEP 1: Background Subtraction & Low-NDVI & Texture Masks
	// ------------------------------------------------------------------------
	Mat nonConcreteMask = computeNonConcreteMask(rgbImg);
	Mat smoothTextureMask = computeSmoothTextureMask(rgbImg, params.textureThresh);
	Mat ndviMask = computeLowNDVIMask(bands, rgbImg.size(), (float)params.ndviThresh);

	Mat candidateMask;
	if (!ndviMask.empty()) {
		// Primary route: Combine Low-NDVI with Non-Concrete and Texture filters
		bitwise_and(nonConcreteMask, ndviMask, candidateMask);
		bitwise_and(candidateMask, smoothTextureMask, candidateMask);
	} else {
		// Fallback route (RGB only): Non-Concrete + Texture + Color
		Mat rgbf; rgbImg.convertTo(rgbf, CV_32F);
		vector<Mat> bgr; split(rgbf, bgr);
		Mat PlantIndex = bgr[1] + bgr[2] - 2.0f * bgr[0]; // G + R - 2B

		Mat PlantIndex_norm, PlantIndex8;
		normalize(PlantIndex, PlantIndex_norm, 0.0f, 255.0f, NORM_MINMAX);
		PlantIndex_norm.convertTo(PlantIndex8, CV_8U);

		Mat rgbColorMask;
		threshold(PlantIndex8, rgbColorMask, 0, 255, THRESH_BINARY | THRESH_OTSU);

		bitwise_and(nonConcreteMask, rgbColorMask, candidateMask);
		bitwise_and(candidateMask, smoothTextureMask, candidateMask);
	}

	// ------------------------------------------------------------------------
	// STEP 1b: ROI Mask From Green Centroid Radius (drop border noise/out-of-focus)
	// ------------------------------------------------------------------------
	if (params.centroidRadiusX > 0 && params.centroidRadiusY > 0) {
		Mat roiMask = Mat::zeros(candidateMask.size(), CV_8U);
		int cx = params.centroidCenterX >= 0 ? params.centroidCenterX : candidateMask.cols / 2;
		int cy = params.centroidCenterY >= 0 ? params.centroidCenterY : candidateMask.rows / 2;
		ellipse(roiMask, Point(cx, cy),
		        Size(params.centroidRadiusX, params.centroidRadiusY), 0, 0, 360, Scalar(255), FILLED);
		bitwise_and(candidateMask, roiMask, candidateMask);
	}

	// ------------------------------------------------------------------------
	// STEP 2: Stronger Morphological Opening (Erosion before Dilation)
	// ------------------------------------------------------------------------
	// Median blur to remove point noise
	medianBlur(candidateMask, candidateMask, params.medianBlurSize);

	// OPENING (Erosion -> Dilation) with 7x7 kernel: Destroys fine moss bridges
	Mat kernelOpen = getStructuringElement(MORPH_ELLIPSE, Size(params.openKernelSize, params.openKernelSize));
	morphologyEx(candidateMask, candidateMask, MORPH_OPEN, kernelOpen);

	// CLOSING with 5x5 kernel: Fills small internal leaf gaps
	Mat kernelClose = getStructuringElement(MORPH_ELLIPSE, Size(params.closeKernelSize, params.closeKernelSize));
	morphologyEx(candidateMask, candidateMask, MORPH_CLOSE, kernelClose);

	// ------------------------------------------------------------------------
	// STEP 3: Dynamic Adaptive Area & Solidity / Compactness Filtering
	// ------------------------------------------------------------------------
	vector<vector<Point>> contours;
	findContours(candidateMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// Dynamic Minimum Area Threshold (0.15% of total image pixels)
	double minAreaThreshold = rgbImg.cols * rgbImg.rows * params.minAreaRatio;

	struct ContourData {
		int index;
		double area;
		double solidity;
		Point2f centroid;
	};

	vector<ContourData> validContours;
	double totalValidArea = 0;
	Point2f weightedCentroid(0, 0);
	vector<Point> allPlantPoints;

	Mat filteredMask = Mat::zeros(candidateMask.size(), CV_8U);

	for (int i = 0; i < (int)contours.size(); i++) {
		double area = contourArea(contours[i]);

		if (area >= minAreaThreshold) {
			// Calculate Solidity = Area / Convex Hull Area
			vector<Point> hull;
			convexHull(contours[i], hull);
			double hullArea = contourArea(hull);
			double solidity = (hullArea > 0) ? (area / hullArea) : 0.0;

			// Solidity filter: Moss/scattered noise < 0.50, Foliage/Canopy >= 0.50
			if (solidity >= params.solidityThresh) {
				Moments mu = moments(contours[i]);
				Point2f centroid(mu.m10 / (mu.m00 + 1e-6), mu.m01 / (mu.m00 + 1e-6));

				validContours.push_back({i, area, solidity, centroid});

				// Draw valid plant contour to final mask
				drawContours(filteredMask, contours, i, Scalar(255), FILLED);

				totalValidArea += area;
				weightedCentroid += centroid * (float)area;

				for (const auto& p : contours[i]) {
					allPlantPoints.push_back(p);
				}
			}
		}
	}

	// ------------------------------------------------------------------------
	// STEP 3b: Clustering Refinement (K-Means then Spatial) on valid contours
	// ------------------------------------------------------------------------
	// Spectral K-Means first: splits spectrally-similar big moss blobs from the
	// true canopy using [Hue, Saturation, NDVI]; then Spatial clustering drops any
	// remaining satellite blobs physically detached from the primary cluster.
	if ((params.spectralKMeans || params.spatialCluster) && totalValidArea > 0) {
		// Build float NDVI image once, reuse for the spectral classifier
		Mat ndviImg;
		const float eps = 1e-6f;
		if (bands.count(5) && (bands.count(3) || bands.count(2))) {
			Mat NIR = bands.at(5).clone(), red = bands.count(3) ? bands.at(3).clone() : bands.at(2).clone();
			if (NIR.channels() == 1 && red.channels() == 1) {
				resize(NIR, NIR, filteredMask.size());
				resize(red, red, filteredMask.size());
				Mat NIRf, redf;
				NIR.convertTo(NIRf, CV_32F); red.convertTo(redf, CV_32F);
				ndviImg = (NIRf - redf) / (NIRf + redf + eps);
			}
		}

		if (params.spectralKMeans) {
			Mat refined = refineMaskWithKMeans(filteredMask, rgbImg, ndviImg);
			if (!refined.empty()) filteredMask = refined;
		}

		if (params.spatialCluster) {
			Mat cleaned = filterSatelliteClusters(filteredMask, (double)params.spatialMergeDist);
			if (!cleaned.empty()) filteredMask = cleaned;
		}

		// Recompute aggregate stats on the refined mask
		totalValidArea = 0;
		weightedCentroid = Point2f(0, 0);
		allPlantPoints.clear();
		findContours(filteredMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (const auto& c : contours) {
			double a = contourArea(c);
			if (a > 0) {
				Moments mu = moments(c);
				Point2f ctd(mu.m10 / (mu.m00 + 1e-6), mu.m01 / (mu.m00 + 1e-6));
				totalValidArea += a;
				weightedCentroid += ctd * (float)a;
				for (const auto& p : c) allPlantPoints.push_back(p);
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
			convexHull(allPlantPoints, results.convexHull);
			results.ellipse = fitEllipse(allPlantPoints);
		}
	} else {
		results.mask = Mat::zeros(candidateMask.size(), CV_8U);
		results.valid = false;
	}

	// Apply final mask to vegetation index image
	if (!indexImg.empty() && indexImg.size() == results.mask.size()) {
		indexImg.setTo(0, results.mask == 0);
	}

	return results;
}

void exportVegIndexCsv(const string& outPath, const vector<string>& requestedIndices, const map<string, map<string, double>>& averages) {
	string csvPath = outPath + "/vegetation_index_report.csv";
	ofstream csv(csvPath);
	if (!csv.is_open()) return;
	csv << fixed << setprecision(6) << "Group prefix";
	for (const string& idx : requestedIndices) { string upperIdx = idx; transform(upperIdx.begin(), upperIdx.end(), upperIdx.begin(), ::toupper); csv << "," << upperIdx << "_Avg"; }
	csv << endl;
	for (const auto& [prefix, indexMap] : averages) {
		csv << prefix;
		for (const string& idx : requestedIndices) { auto it = indexMap.find(idx); csv << "," << (it != indexMap.end() ? it->second : 0.0); }
		csv << endl;
	}
	csv.close();
}
