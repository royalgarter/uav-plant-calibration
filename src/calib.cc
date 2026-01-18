#include <iostream>
#include <filesystem>
#include <regex>
#include <sstream>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp> // For findTransformECC
#include <tiffio.h>

using namespace std;
using namespace std::filesystem;
using namespace cv;

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

ImageInfo parseMetadata(const string& filePath) {
    ImageInfo info;
    info.path = filePath;
    info.filename = path(filePath).filename().string();
    info.ext = info.filename.substr(info.filename.find_last_of(".") + 1);

    TIFF* tif = TIFFOpen(filePath.c_str(), "r");
    if (tif) {
        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &info.width);
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &info.height);

        void* data;
        uint32_t len;
        if (TIFFGetField(tif, 700, &len, &data)) {
            string xml((char*)data, len);
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
        TIFFClose(tif);
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

int main(int argc, char** argv) {
    cout << "calib" << endl;

    string inDir = (argc > 1) ? argv[1] : "input";
    string outDir = (argc > 2) ? argv[2] : "output";

    if (!exists(inDir)) return 1;
    create_directories(outDir);

    vector<ImageInfo> allImages;
    cout << "Scanning " << inDir << "..." << endl;
    for (const auto& entry : directory_iterator(inDir)) {
        string path = entry.path().string();
        string ext = entry.path().extension().string(); 
        if (path.find(".tif") == string::npos && path.find(".TIF") == string::npos) continue;

        allImages.push_back(parseMetadata(path));
    }

    // Group by UUID
    map<string, vector<ImageInfo>> groups;
    for (const auto& info : allImages) {
        if (!info.uuid.empty()) {
            groups[info.uuid].push_back(info);
        } else {
            groups["unknown"].push_back(info);
        }
    }

    for (auto& [uuid, group] : groups) {
        cout << "Processing group: " << uuid << " (" << group.size() << " images)" << endl;

        ImageInfo* refInfo = nullptr;
        Mat refMat;
        
        for (auto& info : group) {
            if (abs(info.relX) < 0.001 && abs(info.relY) < 0.001) {
                cout << info.filename << ", " << info.relX << ", " << info.relY << '\n';
                refInfo = &info;
                break;
            }
        }

        if (refInfo) {
            cout << "  Reference found: " << refInfo->filename << endl;
            Mat rawRef = imread(refInfo->path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
            if (!rawRef.empty()) {
                refMat = undistortImg(rawRef, *refInfo);
            }
        } else {
            cout << "  No reference image found for group " << uuid << endl;
        }

        for (auto& info : group) {
            Mat raw = imread(info.path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
            if (raw.empty()) continue;

            Mat dewarped = undistortImg(raw, info);
            Mat finalImg;

            // --- STEP B: INITIAL ALIGNMENT (Metadata) ---
            Mat H_meta = Mat::eye(3, 3, CV_64F);
            if (info.foundH) {
                H_meta = info.H;
            } else if (abs(info.relX) > 0.0001 || abs(info.relY) > 0.0001) {
                // Translation
                H_meta.at<double>(0, 2) = info.relX;
                H_meta.at<double>(1, 2) = info.relY;
            }

            // --- STEP C: OPTIONAL FINE TUNING (ECC) ---
            Mat H_total = H_meta.clone();
            
            if (refInfo && refInfo->path != info.path && !refMat.empty()) {
                cout << "  Aligning " << info.filename << " to " << refInfo->filename << " using ECC..." << endl;
                
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
                    cout << "    ECC converged (cc=" << cc << ")" << endl;

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
                    cerr << "    ECC failed: " << e.what() << endl;
                }
            }

            cout << "  Saving " << info.filename << endl;
            warpPerspective(dewarped, finalImg, H_total, dewarped.size(), INTER_LINEAR | WARP_INVERSE_MAP);
            imwrite(outDir + "/" + info.filename, finalImg);
        }
    }
    return 0;
}