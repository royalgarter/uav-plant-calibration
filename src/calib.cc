#include "calib.h"
#include <iomanip>
#include <ctime>
#include <sstream>
#include <omp.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>

// Uncomment to enable Windows GUI support
// #define WINGUI

#ifdef WINGUI
#include "calib-raygui.cc"
#endif

using namespace std;
using namespace std::filesystem;
using namespace cv;

void showUsage() {
	cout << "USAGE: ./calib <src_dir (default: .input/)> <dest_dir (default: .output/)> [--radio] [--auto] [--optimize] [--veg-idx=...]" << endl;
	cout << "  --radio       Enable radiometric calibration." << endl;
	cout << "  --auto        Auto-detect radiometric board (used with --radio). Optional: --auto <border_thickness>" << endl;
	cout << "  --template    Path to radiometric board template image (default: .input_ref/radiometric_board.jpg)" << endl;
	cout << "  --ref         Path to radiometric reference CSV file (default: .input_ref/radiometric_reference.csv)" << endl;
	cout << "  --optimize    Enable performance optimizations for ECC alignment." << endl;
	cout << "                Optional: --optimize <downscale>,<iterations>,<epsilon>" << endl;
	cout << "                Default: --optimize 2,50,1e-4" << endl;
	cout << "  --veg-idx     Comma-separated list of vegetation indices to calculate." << endl;
	cout << "                Supported: ndvi, sr, rsr, arvi, afri1.6, afri2.1, vari, tvi, msarvi, gemi," << endl;
	cout << "                           evi, evi2, mcari, osavi, savi, msavi, dvi, nri, ndre, gndvi," << endl;
	cout << "                           cigreen, cire, ipvi, rdvi, wdrvi, wdvi, tsavi, atsavi, msr" << endl;
	cout << "                Default: ndvi" << endl;
	cout << "  --green-centroid-radius, -gcr  Radius in pixels to focus green mask around image center (default: 0 = disabled)" << endl;
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
	vector<string> requestedVegIndices = {"ndvi"};
	// Preprocessing (green mask noise filtering) parameters
	GreenMaskParams greenParams;

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
			} else if (arg.find("--veg-idx=") == 0) {
				string list = arg.substr(10);
				stringstream ss(list);
				string segment;
				requestedVegIndices.clear();
				while(getline(ss, segment, ',')) {
					transform(segment.begin(), segment.end(), segment.begin(), ::tolower);
					requestedVegIndices.push_back(segment);
				}
			} else if (arg == "--veg-idx" && i + 1 < argc && argv[i+1][0] != '-') {
				string list = argv[i+1];
				stringstream ss(list);
				string segment;
				requestedVegIndices.clear();
				while(getline(ss, segment, ',')) {
					transform(segment.begin(), segment.end(), segment.begin(), ::tolower);
					requestedVegIndices.push_back(segment);
				}
				i++;
			} else if (arg == "--green-centroid-radius" || arg == "-gcr") {
				if (i + 1 < argc && argv[i+1][0] != '-') {
					string val = argv[i+1];
					auto commaPos = val.find(',');
					if (commaPos != string::npos) {
						string sx = val.substr(0, commaPos);
						string sy = val.substr(commaPos + 1);
						try {
							greenParams.centroidRadiusX = stoi(sx);
							greenParams.centroidRadiusY = stoi(sy);
						} catch (...) {
							// ignore parse error, keep defaults
						}
					} else {
						try {
							int r = stoi(val);
							greenParams.centroidRadiusX = greenParams.centroidRadiusY = r;
						} catch (...) {
							// ignore
						}
					}
					i++;
				}
			} else if (arg == "--texture-thresh") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.textureThresh = atof(argv[i+1]); i++; }
			} else if (arg == "--ndvi-thresh") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.ndviThresh = atof(argv[i+1]); i++; }
			} else if (arg == "--min-area-ratio") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.minAreaRatio = atof(argv[i+1]); i++; }
			} else if (arg == "--solidity-thresh") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.solidityThresh = atof(argv[i+1]); i++; }
			} else if (arg == "--median-blur") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.medianBlurSize = atoi(argv[i+1]); i++; }
			} else if (arg == "--open-kernel") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.openKernelSize = atoi(argv[i+1]); i++; }
			} else if (arg == "--close-kernel") {
				if (i + 1 < argc && argv[i+1][0] != '-') { greenParams.closeKernelSize = atoi(argv[i+1]); i++; }
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
	string alignDir = outDir + "/alignment";
	string radioDir = outDir + "/radiometric";
	string vegidxDir = outDir + "/vegetation_index";

	create_directories(alignDir);
	create_directories(radioDir);
	create_directories(vegidxDir);

	gLog << "Calibrated images: " << alignDir << endl;
	gLog << "Radiometric images: " << radioDir << endl;
	gLog << "NDVI images: " << vegidxDir << endl;
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
				Mat rawRef = imread(targetInfo->path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
				if (!rawRef.empty()) {
					// Use undistorted image for board detection (Step 2 needs consistent geometry)
					Mat dewarpedRef = undistortImg(rawRef, *targetInfo);
					data.coeffs = getRadiometricCoeffs(dewarpedRef, targetInfo->filename, radioInterval, autoRadioThickness, radioDir, radioTemplatePath);
					
					// Store anchor points from the reference image
					Point p56 = data.coeffs.p56;
					Point p3 = data.coeffs.p3;
					int boxSize = data.coeffs.boxSize;
					bool isRGB = data.coeffs.isRGB;
					
					if (data.coeffs.valid) {
						gLog << "  Collecting DN values from all images in group..." << endl;
						
						// Collect DN values from all images in the group
						for (auto& info : data.images) {
							Mat raw = imread(info.path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
							if (raw.empty()) continue;

							// Align to reference image geometry before collecting DNs
							Mat dewarped = undistortImg(raw, info);
							
							Mat H_meta = Mat::eye(3, 3, CV_64F);
							if (info.foundH) {
								H_meta = info.H;
							} else if (abs(info.relX) > 0.0001 || abs(info.relY) > 0.0001) {
								H_meta.at<double>(0, 2) = info.relX;
								H_meta.at<double>(1, 2) = info.relY;
							}

							double scaleX = (double)dewarpedRef.cols / dewarped.cols;
							double scaleY = (double)dewarpedRef.rows / dewarped.rows;
							Mat H_meta_ref = H_meta.clone();
							H_meta_ref.col(0) *= (1.0 / scaleX);
							H_meta_ref.col(1) *= (1.0 / scaleY);

							Mat aligned;
							warpPerspective(dewarped, aligned, H_meta_ref, dewarpedRef.size(), INTER_LINEAR | WARP_INVERSE_MAP);
							
							string stem = path(info.filename).stem().string();
							if (stem.empty()) continue;
							char lastChar = stem.back();
							
							if (lastChar == '0') {
								// RGB image - collect per-channel DN values
								vector<double> dns_r, dns_g, dns_b;
								collectDnValues(aligned, p56, p3, boxSize, true, dns_r, dns_g, dns_b);
								data.rgbDns_r = dns_r;
								data.rgbDns_g = dns_g;
								data.rgbDns_b = dns_b;
								data.rgbCollected = true;
								gLog << "    Collected RGB DN values from " << info.filename << endl;
							} else {
								// Multispectral image - collect single-band DN values
								int band = lastChar - '0';
								vector<double> dns_r, dns_g, dns_b;
								collectDnValues(aligned, p56, p3, boxSize, false, dns_r, dns_g, dns_b);
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

	map<string, map<string, double>> groupVegAverages;

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
				// if (doRadio && data.coeffs.valid) {
				// 	processedRef = applyRadiometricCalibration(rawRef, data.coeffs);
				// }
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
			// if (doRadio && data.coeffs.valid) {
			// 	processed = applyRadiometricCalibration(raw, data.coeffs);
			// }

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
			imwrite(alignDir + "/" + info.filename + ".jpg", tmpImg);

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
				if (band >= 0 && band <= 9) {
					#pragma omp critical
					{
						alignedBands[band] = finalImg.clone();
					}
				}
			}
			gLog << ss.str();
		}

		// Pre-calculate common green mask if RGB image (band 0) is available
		GreenMaskResults commonRes;
		Mat commonMask;
		bool hasCommonMask = false;
		if (alignedBands.count(0)) {
			Mat dummy;
			commonRes = applyGreenMask(dummy, alignedBands[0], vegidxDir, prefix, "common", alignedBands, greenParams);
			commonMask = commonRes.mask;
			hasCommonMask = true;
		}

		// Calculate and save requested vegetation indices
		for (const string& vegIdx : requestedVegIndices) {
			gLog << "\n    Step D: Calculating " << vegIdx << " for group " << prefix << "..." << endl;
			clock_t startD = clock();

			imwrite(vegidxDir + "/" + prefix + "_mask.jpg", commonMask);

			Mat indexImg = calculateVegIndex(vegIdx, alignedBands, commonMask);
			
			if (indexImg.empty()) {
				stringstream ssBands;
				ssBands << "    Warning: Missing bands for " << vegIdx << " calculation. Available bands: ";
				for (auto const& [b, m] : alignedBands) ssBands << b << " ";
				gLog << ssBands.str() << endl;
				continue;
			}

			// Apply green mask results if available
			if (hasCommonMask) {
				Mat greenMask = commonMask;

				// Create RGB composite: overlay semi-transparent green where mask is true
				Mat rgbImg = alignedBands[0];
				Mat rgb_u8;
				if (rgbImg.type() != CV_8UC3) rgbImg.convertTo(rgb_u8, CV_8UC3);
				else rgb_u8 = rgbImg.clone();
				Mat overlay(rgb_u8.size(), CV_8UC3, Scalar(0, 255, 0)); // BGR green overlay
				Mat blended;
				addWeighted(rgb_u8, 0.5, overlay, 0.5, 0.0, blended);
				Mat composite = rgb_u8.clone();
				blended.copyTo(composite, greenMask);

				// Draw extraction results for verification
				if (commonRes.valid) {
					// Draw Convex Hull (Yellow)
					if (!commonRes.convexHull.empty()) {
						vector<vector<Point>> hulls = { commonRes.convexHull };
						drawContours(composite, hulls, 0, Scalar(0, 255, 255), 2);
					}
					// Draw Ellipse (Red)
					if (commonRes.ellipse.size.width > 0) {
						ellipse(composite, commonRes.ellipse, Scalar(0, 0, 255), 2);
					}
					// Draw Centroid (Blue Cross)
					int cs = 10;
					line(composite, Point(commonRes.centroid.x - cs, commonRes.centroid.y), Point(commonRes.centroid.x + cs, commonRes.centroid.y), Scalar(255, 0, 0), 2);
					line(composite, Point(commonRes.centroid.x, commonRes.centroid.y - cs), Point(commonRes.centroid.x, commonRes.centroid.y + cs), Scalar(255, 0, 0), 2);
				}

				if (vegIdx == requestedVegIndices[0]) { // Save mask preview once
					imwrite(vegidxDir + "/" + prefix + "_green_mask.tif", composite);
				}
			}

			// Calculate average (excluding 0/masked pixels)
			Scalar avgVal = mean(indexImg, indexImg > 0);
			groupVegAverages[prefix][vegIdx] = avgVal[0];

			// Save raw float index
			imwrite(vegidxDir + "/" + prefix + "_" + vegIdx + "_raw.tif", indexImg);

			// Save colorized index
			Mat index_u8 = contrastStretch(indexImg);
			imwrite(vegidxDir + "/" + prefix + "_" + vegIdx + "_u8.jpg", index_u8);

			Mat index_color;
			applyColorMap(index_u8, index_color, COLORMAP_JET);
			imwrite(vegidxDir + "/" + prefix + "_" + vegIdx + "_color.jpg", index_color);

			gLog << "    " << vegIdx << " saved to " << prefix + "_" + vegIdx + "_raw.tif and " << prefix + "_" + vegIdx + "_color.jpg" << endl;
			gLog << "    Step D (" << vegIdx << ") Total: " << fixed << setprecision(2) << double(clock() - startD) / CLOCKS_PER_SEC << "s" << endl;
		}
	}

	// Export vegetation index report
	if (!requestedVegIndices.empty()) {
		gLog << "\n--- EXPORTING VEGETATION INDEX CSV ---" << endl;
		exportVegIndexCsv(outDir, requestedVegIndices, groupVegAverages);
	}

	if (useGui) {
		cout << "Press ENTER to exit..." << endl;
		cin.get();
	}

	return 0;
}