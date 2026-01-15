#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <filesystem>
#include <string>
#include <algorithm>
#include <cctype>

using namespace std;
using namespace cv;

namespace fs = filesystem;
using namespace cv::fisheye;

// #define WINGUI

#ifdef WINGUI
#include "wingui.cc"
#endif

// --- EXISTING CALIBRATION LOGIC ---

vector<Point3f> calibratePattern(Size checkboardSize, float squareSize) {
	vector<Point3f> ret;
	for (int i = 0; i < checkboardSize.height; i++) {
		for (int j = 0; j < checkboardSize.width; j++) {
			ret.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
	}
	return ret;
}

string promptForInput(const string& message) {
	cout << message;
	string input;
	getline(cin, input);
	return input;
}

bool usage() {
	cout << "USAGE: ./fisheye <src_dir> <dest_dir> <checkboard_dir> <checkboard_width> <checkboard_height>" << endl;
	cout << "   Or: ./fisheye <src_dir> <dest_dir> <calibration_file> <checkboard_width> <checkboard_height> (Export Mode)" << endl;
	cout << "   Or: ./fisheye <src_dir> <dest_dir> <calibration_file> (Import Mode)" << endl;
	cout << "   Or: ./fisheye -i (Interactive Mode)" << endl;
	cout << "   Or: ./fisheye (Default Interactive Mode)" << endl;
	// cout << "   Or: ./fisheye -gui (Window Mode)" << endl;

	cout << "---" << endl;

	return 1;
}

int main(int argc, char** argv) {
	string srcPath, destPath, samplesDir, configFile, param3;
	int checkboardWidth = 0, checkboardHeight = 0;
	bool useGui = false, useTui = false;
	bool calibrationNeeded = true; // Default for GUI/Interactive
	bool exportCalibration = false;
	bool importCalibration = false;

	auto tik = chrono::high_resolution_clock::now();
	auto tok = chrono::high_resolution_clock::now();

	// Check for GUI flag
	if (argc > 1 && string(argv[1]) == "-gui") {
		useGui = true;
	}

	if (useGui) {
		#ifdef WINGUI
		cout << "Launching GUI..." << endl;
		// The main function will not exit until the GUI window is closed.
		// It relies on g_guiSubmitted and g_guiCancelled globals set by WndProc.
		if (!runWinGuiMode(srcPath, destPath, param3, checkboardWidth, checkboardHeight)) {
			cout << "GUI cancelled or exited." << endl;
			return 0; // Exit if GUI was cancelled or failed to initialize
		}
		#endif
	} else if (argc < 2 || (argc > 1 && string(argv[1]) == "-i")) {
		usage();

		cout << "Entering interactive mode. Please provide the following inputs:" << endl;

		srcPath = promptForInput("1. Enter source directory: ");
		destPath = promptForInput("2. Enter destination directory: ");
		param3 = promptForInput("3. Enter checkboard samples directory: ");

		checkboardWidth = stoi(promptForInput("4. Enter checkboard width: "));
		checkboardHeight = stoi(promptForInput("5. Enter checkboard height: "));

		useTui = true;
	} else {
		if (argc < 4) {
			return usage();
		}

		srcPath = argv[1];
		destPath = argv[2];
		param3 = argv[3];
	}

	if (fs::is_directory(param3)) {
		// Case 1: Folder path (Standard Calibration)
		samplesDir = param3;

		if (useTui == false) {
			checkboardWidth = stoi(argv[4]);
			checkboardHeight = stoi(argv[5]);
		}

		if (checkboardWidth == 0 || checkboardHeight == 0) {
			cerr << "Error: Missing width/height for calibration folder mode." << endl;
			return 1;
		}

		calibrationNeeded = true;
	} else {
		// Case 2: Filename
		configFile = param3;
		if (argc >= 6) {
			// Case 2.1: Export
			// Assuming srcPath is the sample source as well since samplesDir arg is used for config
			samplesDir = filesystem::path(configFile).parent_path().string();

			if (useTui == false) {
				checkboardWidth = stoi(argv[4]);
				checkboardHeight = stoi(argv[5]);
			}

			calibrationNeeded = true;
			exportCalibration = true;
		} else {
			// Case 2.2: Import
			calibrationNeeded = false;
			importCalibration = true;
		}
	}

	Matx33d K;
	Vec4d D;

	if (calibrationNeeded) {
		// 1. Load Calibration Images
		vector<Mat> images;
		cout << "Loading samples from " << samplesDir << "..." << endl;

		if (!fs::exists(samplesDir)) {
			cerr << "Error: Samples directory '" << samplesDir << "' not found." << endl;
			return 1;
		}

		for (const auto& entry : fs::directory_iterator(samplesDir)) {
			string p = entry.path().string();
			string ext = entry.path().extension().string();
			// Convert ext to lowercase for comparison
			transform(ext.begin(), ext.end(), ext.begin(),
						   [](unsigned char c){ return tolower(c); });

			// Simple check for jpg/png
			if (ext == ".jpg" || ext == ".png" || ext == ".jpeg" || ext == ".bmp") {
				Mat img = imread(p, IMREAD_GRAYSCALE);
				if (!img.empty()) {
					images.push_back(img);
				}
			}
		}

		if (images.empty()) {
			cerr << "No images found in " << samplesDir << endl;
			if (useGui) system("pause"); // Keep window open to see error
			return 1;
		}
		cout << "Loaded " << images.size() << " images." << endl;

		// 2. Calibrate
		cout << "Calibrating..." << endl;
		Size checkboardSize(checkboardWidth, checkboardHeight);
		vector<vector<Point3f>> objPoints;
		vector<Mat> imgPoints;
		vector<Point3f> pattern = calibratePattern(checkboardSize, 1.0);
		TermCriteria subpixCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1);

		for (auto& img : images) {
			Mat corners;
			// Use CALIB_CB_ADAPTIVE_THRESH for better robustness
			tik = chrono::high_resolution_clock::now();
			bool found = findChessboardCorners(img, checkboardSize, corners,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

			if (found) {
				objPoints.push_back(pattern);
				cornerSubPix(img, corners, Size(3, 3), Size(-1, -1), subpixCriteria);
				imgPoints.push_back(corners);
			}

			tok = chrono::high_resolution_clock::now();
			cout << "findChessboardCorners: " << chrono::duration_cast<chrono::milliseconds>(tok - tik).count() << endl;
		}

		if (objPoints.empty()) {
			cerr << "Could not detect any checkboards with size " << checkboardWidth << "x" << checkboardHeight << endl;
			if (useGui) system("pause");
			return 1;
		}

		Size size = images[0].size();
		int flag = CALIB_RECOMPUTE_EXTRINSIC | CALIB_CHECK_COND | CALIB_FIX_SKEW;
		TermCriteria criteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 1e-6);

		tik = chrono::high_resolution_clock::now();
		double error = calibrate(objPoints, imgPoints, size, K, D, noArray(), noArray(), flag, criteria);
		tok = chrono::high_resolution_clock::now();

		cout << "Calibration done. Reprojection error: " << error << "Time: " << chrono::duration_cast<chrono::milliseconds>(tok - tik).count() << endl;

		if (exportCalibration) {
			cout << "Exporting calibration data to " << configFile << "..." << endl;
			ofstream out(configFile);
			if (out.is_open()) {
				out << K(0, 0) << " " << K(1, 1) << " " << K(0, 2) << " " << K(1, 2) << endl;
				out << D[0] << " " << D[1] << " " << D[2] << " " << D[3] << endl;
				out.close();
				cout << "Export successful." << endl;
			} else {
				cerr << "Error: Could not open file for writing: " << configFile << endl;
			}
		}

	} else if (importCalibration) {
		cout << "Importing calibration data from " << configFile << "..." << endl;
		ifstream in(configFile);
		if (in.is_open()) {
			double fx, fy, cx, cy;
			if (in >> fx >> fy >> cx >> cy) {
				K = Matx33d::eye();
				K(0, 0) = fx; K(1, 1) = fy; K(0, 2) = cx; K(1, 2) = cy;
			} else {
				 cerr << "Error reading K matrix." << endl;
				 return 1;
			}
			if (!(in >> D[0] >> D[1] >> D[2] >> D[3])) {
				 cerr << "Error reading D coefficients." << endl;
				 return 1;
			}
			in.close();
			cout << "Import successful." << endl;
		} else {
			cerr << "Error: Could not open file for reading: " << configFile << endl;
			return 1;
		}
	}

	// 3. Undistort & 4. Save
	cout << "Undistorting images from " << srcPath << " to " << destPath << "..." << endl;

	if (!fs::exists(srcPath) || !fs::is_directory(srcPath)) {
		cerr << "Error: Source path '" << srcPath << "' is not a directory." << endl;
		if (useGui) system("pause");
		return 1;
	}

	if (!fs::exists(destPath)) {
		if (!fs::create_directories(destPath)) {
			cerr << "Error: Could not create destination directory '" << destPath << "'." << endl;
			if (useGui) system("pause");
			return 1;
		}
	} else if (!fs::is_directory(destPath)) {
		cerr << "Error: Destination path '" << destPath << "' is not a directory." << endl;
		if (useGui) system("pause");
		return 1;
	}

	int count = 0;
	for (const auto& entry : fs::directory_iterator(srcPath)) {
		string p = entry.path().string();
		if (!entry.is_regular_file()) continue;

		string ext = entry.path().extension().string();
		string filename = entry.path().stem().string();

		string ext_lower = ext;
		transform(ext_lower.begin(), ext_lower.end(), ext_lower.begin(),
					   [](unsigned char c){ return tolower(c); });

		if (ext_lower == ".jpg" || ext_lower == ".png" || ext_lower == ".jpeg" || ext_lower == ".bmp") {
			Mat distorted = imread(p);
			if (distorted.empty()) {
				cerr << "Failed to read image: " << p << endl;
				continue;
			}

			Mat undistorted;
			// K is used for both original and new camera matrix to keep the scale
			tik = chrono::high_resolution_clock::now();
			undistortImage(distorted, undistorted, K, D, K, distorted.size());
			tok = chrono::high_resolution_clock::now();

			cout << "Time: " << chrono::duration_cast<chrono::milliseconds>(tok - tik).count() << endl;

			string outFilename = filename + "_undistored" + ext;
			fs::path outPath = fs::path(destPath) / outFilename;

			if (imwrite(outPath.string(), undistorted)) {
				cout << "Saved to " << outPath.string() << endl;
				count++;
			} else {
				cerr << "Failed to save to " << outPath.string() << endl;
			}
		}
	}
	cout << "Processed " << count << " images." << endl;

	if (useGui) system("pause");
	return 0;
}

