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

Mat calculateVegIndex(const string& type, const map<int, Mat>& bands) {
	Mat nir, red, green, blue, re;
	if (bands.count(5)) bands.at(5).convertTo(nir, CV_32F);
	else if (bands.count(4)) bands.at(4).convertTo(nir, CV_32F);
	if (bands.count(3)) bands.at(3).convertTo(red, CV_32F);
	else if (bands.count(2) && !bands.count(3)) bands.at(2).convertTo(red, CV_32F);
	if (bands.count(2)) bands.at(2).convertTo(green, CV_32F);
	else if (bands.count(1) && !bands.count(2)) bands.at(1).convertTo(green, CV_32F);
	if (bands.count(1)) bands.at(1).convertTo(blue, CV_32F);
	if (bands.count(4)) bands.at(4).convertTo(re, CV_32F);
	else if (bands.count(3) && !bands.count(4)) bands.at(3).convertTo(re, CV_32F);

	Mat result;
	if (type == "ndvi") { if (nir.empty() || red.empty()) return Mat(); result = (nir - red) / (nir + red + 1e-6f); }
	else if (type == "evi") { if (nir.empty() || red.empty() || blue.empty()) return Mat(); result = 2.5f * (nir - red) / (nir + 6.0f * red - 7.5f * blue + 1.0f); }
	else if (type == "gndvi") { if (nir.empty() || green.empty()) return Mat(); result = (nir - green) / (nir + green + 1e-6f); }
	else if (type == "ndre") { if (nir.empty() || re.empty()) return Mat(); result = (nir - re) / (nir + re + 1e-6f); }
	else if (type == "rdvi") { if (nir.empty() || red.empty()) return Mat(); Mat sum_val = nir + red; sqrt(sum_val, sum_val); result = (nir - red) / (sum_val + 1e-6f); }
	else if (type == "osavi") { if (nir.empty() || red.empty()) return Mat(); result = 1.16f * (nir - red) / (nir + red + 0.16f); }
	else if (type == "msr") { if (nir.empty() || red.empty()) return Mat(); Mat ratio = nir / (red + 1e-6f); Mat sqrt_ratio; sqrt(ratio, sqrt_ratio); result = (ratio - 1.0f) / (sqrt_ratio + 1.0f + 1e-6f); }

	if (!result.empty()) { threshold(result, result, 0, 0, THRESH_TOZERO); threshold(result, result, 1, 1, THRESH_TRUNC); }
	return result;
}

cv::Mat applyGreenMask(cv::Mat& indexImg, const cv::Mat& rgbImg, const string& outputDir, const string& prefix, const string& indexName, const map<int, Mat>& bands, int greenCentroidRadius) {
	if (rgbImg.empty() || indexImg.empty()) return Mat();

	gLog << "  INFO: greenCentroidRadius: " << greenCentroidRadius << endl;

	// Convert BGR to float channels
	Mat rgbf; rgbImg.convertTo(rgbf, CV_32F);
	vector<Mat> bgr; split(rgbf, bgr);
	Mat B = bgr[0]; Mat G = bgr[1]; Mat R = bgr[2];
	const float eps = 1e-6f;

	// 1. MODIFIED VEGETATION INDEX:
    // Instead of Excess Green (2G-R-B), we use an index that rewards BOTH Green and Red (Yellow = Green + Red)
    // while heavily penalizing Blue (which is present in the gray concrete shadows).
	Mat PlantIndex = G + R - 2.0f * B;

	// Normalize PlantIndex for thresholding
	Mat PlantIndex_norm; normalize(PlantIndex, PlantIndex_norm, 0.0f, 255.0f, NORM_MINMAX);
	Mat PlantIndex8; PlantIndex_norm.convertTo(PlantIndex8, CV_8U);

	// Otsu threshold on the new Plant Index
	Mat maskPlantIndex; threshold(PlantIndex8, maskPlantIndex, 0, 255, THRESH_BINARY | THRESH_OTSU);

	// Prepare combined mask; try to use NIR-based GNDVI if available
	Mat combined_mask = Mat::zeros(PlantIndex8.size(), CV_8U);
	bool hasNIR = false;
	Mat maskGNDVI;
	Mat GNDVI_norm;

	if (bands.count(5)) {
		Mat NIR = bands.at(5).clone();
		if (!NIR.empty()) {
			if (NIR.channels() > 1) {
				double minVal, maxVal; minMaxLoc(NIR, &minVal, &maxVal);
				cout << "    Warning: Band 5 has unexpected properties. Ignoring NIR for GNDVI." << endl;
			} else {
				cv::resize(NIR, NIR, PlantIndex8.size());
				Mat NIRf; NIR.convertTo(NIRf, CV_32F);

                if (NIRf.total() > 0) {
					Mat flat = NIRf.reshape(1, (int)NIRf.total());
					Mat sorted;
					cv::sort(flat, sorted, SORT_EVERY_COLUMN + SORT_ASCENDING);
					int total = sorted.rows;
					int idx2 = std::max(0, (int)std::round(total * 0.02f));
					int idx98 = std::min(total - 1, (int)std::round(total * 0.98f));
					float p2 = sorted.at<float>(idx2);
					float p98 = sorted.at<float>(idx98);
					if (p98 <= p2) {
						p2 = sorted.at<float>(0);
						p98 = sorted.at<float>(std::min(total - 1, 1));
					}

					Mat NIRclipped = NIRf.clone();
					Mat lowMask = NIRf < p2;
					Mat highMask = NIRf > p98;
					NIRclipped.setTo(p2, lowMask);
					NIRclipped.setTo(p98, highMask);

					Mat Gf; G.convertTo(Gf, CV_32F);
					Mat GNDVI = (NIRclipped - Gf) / (NIRclipped + Gf + eps);

					normalize(GNDVI, GNDVI_norm, 0.0f, 255.0f, NORM_MINMAX);
					Mat GNDVI8; GNDVI_norm.convertTo(GNDVI8, CV_8U);

					threshold(GNDVI8, maskGNDVI, 0, 255, THRESH_BINARY | THRESH_OTSU);
					hasNIR = true;
				}
			}
		}
	}

	if (hasNIR) {
		Mat score;
		Mat Plant_n32; PlantIndex_norm.convertTo(Plant_n32, CV_32F);
		Mat GNDVI_n32; GNDVI_norm.convertTo(GNDVI_n32, CV_32F);

		float alpha = 0.6f, beta = 0.4f;
		score = alpha * Plant_n32 + beta * GNDVI_n32;

		Mat score8; score.convertTo(score8, CV_8U);
		Mat maskScore; threshold(score8, maskScore, 0, 255, THRESH_BINARY | THRESH_OTSU);

		bitwise_and(maskPlantIndex, maskGNDVI, combined_mask);
		bitwise_or(combined_mask, maskScore, combined_mask);
	} else {
		combined_mask = maskPlantIndex;
	}

	// 2. EXPANDED HSV RANGE:
    // Lowered Hue from 35 to 18 to capture yellow and brownish-green distressed leaves.
    // Lowered Saturation from 40 to 25 because distressed leaves lose their vivid color.
	Mat hsv; cvtColor(rgbImg, hsv, COLOR_BGR2HSV);
	Mat hsv_mask;
    inRange(hsv, Scalar(18, 25, 30), Scalar(90, 255, 255), hsv_mask);

    bitwise_and(combined_mask, hsv_mask, combined_mask);

	// 3. GENTLE MORPHOLOGY:
    // Using a median blur instead of MORPH_OPEN to remove salt-and-pepper concrete noise
    // without erasing the very thin, dying stems of the plant.
    medianBlur(combined_mask, combined_mask, 3);

    // Smaller closure to bridge gaps in thin leaves
	Mat kernelClose = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	morphologyEx(combined_mask, combined_mask, MORPH_CLOSE, kernelClose);

    // Slight dilation to ensure boundary pixels of leaves are kept
	Mat kernelDilate = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	dilate(combined_mask, combined_mask, kernelDilate, Point(-1, -1), 1);

	// 4. FOCUS MASK: either circular centroid radius (preferred) or small border margin fallback
	if (greenCentroidRadius > 0) {
		int w = combined_mask.cols; int h = combined_mask.rows;
		Point center(w/2, h/2);
		Mat circleMask = Mat::zeros(combined_mask.size(), CV_8U);
		int r = greenCentroidRadius;
		// clamp radius to image bounds
		r = std::max(0, std::min(r, std::max(w, h)));
		circle(circleMask, center, r, Scalar(255), FILLED);
		bitwise_and(combined_mask, circleMask, combined_mask);
	} else {
		// small rectangular margin (5%) as a conservative fallback
		int w = combined_mask.cols; int h = combined_mask.rows;
		int marginX = std::max(1, (int)std::round(w * 0.05));
		int marginY = std::max(1, (int)std::round(h * 0.05));
		Mat borderMask = Mat::zeros(combined_mask.size(), CV_8U);
		Rect innerRect(marginX, marginY, std::max(0, w - 2 * marginX), std::max(0, h - 2 * marginY));
		if (innerRect.width > 0 && innerRect.height > 0) borderMask(innerRect).setTo(255);
		bitwise_and(combined_mask, borderMask, combined_mask);
	}

	// Apply mask to index image (zero-out background)
	if (indexImg.size() == combined_mask.size()) {
		indexImg.setTo(0, combined_mask == 0);
	}

	return combined_mask;
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
