#include <iostream>
#include <filesystem>
#include <regex>
#include <sstream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <tiffio.h>

using namespace std;
using namespace std::filesystem;
using namespace cv;

int main(int argc, char** argv) {
    string inDir = (argc > 1) ? argv[1] : "input";
    string outDir = (argc > 2) ? argv[2] : "output";

    if (!exists(inDir)) return 1;
    create_directories(outDir);

    for (const auto& entry : directory_iterator(inDir)) {
        string path = entry.path().string();
        string filename = entry.path().filename().string();
        
        string ext = path.substr(path.find_last_of(".") + 1);
        if (ext != "tif" && ext != "TIF") continue;

        // PDF PAGE 2 & 6: Intrinsic Parameters
        double fx = 0, fy = 0, cx_d = 0, cy_d = 0; // Distortion offsets
        double calibratedCx = 0, calibratedCy = 0; // Base Center (Page 2)
        double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
        bool foundDistortion = false;

        // PDF PAGE 4: Alignment Parameters
        double relX = 0.0, relY = 0.0; 

        Mat H = Mat::eye(3, 3, CV_64F); 
        bool foundH = false;

        TIFF* tif = TIFFOpen(path.c_str(), "r");
        if (tif) {
            void* data; uint32_t len;
            if (TIFFGetField(tif, 700, &len, &data)) { 
                string xml((char*)data, len);
                smatch m;

                // 1. Parse Calibrated Optical Center (PDF Page 2 & 6)
                if (regex_search(xml, m, regex(R"(drone-dji:CalibratedOpticalCenterX=\"([^\"]+)\")"))) 
                    calibratedCx = stod(m[1]);
                if (regex_search(xml, m, regex(R"(drone-dji:CalibratedOpticalCenterY=\"([^\"]+)\")"))) 
                    calibratedCy = stod(m[1]);

                // 2. Parse Relative Optical Center (PDF Page 4 - Step 1)
                // The PDF states these represent the offset to the NIR band.
                if (regex_search(xml, m, regex(R"(drone-dji:RelativeOpticalCenterX=\"([^\"]+)\")"))) 
                    relX = stod(m[1]);
                if (regex_search(xml, m, regex(R"(drone-dji:RelativeOpticalCenterY=\"([^\"]+)\")"))) 
                    relY = stod(m[1]);

                // 3. Parse DewarpData (PDF Page 2)
                if (regex_search(xml, m, regex(R"(drone-dji:DewarpData=\"([^\"]+)\")"))) {
                    string dataStr = m[1];
                    size_t semiPos = dataStr.find(';');
                    if (semiPos != string::npos) {
                        string paramsStr = dataStr.substr(semiPos + 1);
                        stringstream ss(paramsStr);
                        string segment;
                        vector<double> v;
                        while(getline(ss, segment, ',')) v.push_back(stod(segment));
                        
                        if (v.size() >= 9) {
                            fx = v[0]; fy = v[1];
                            cx_d = v[2]; cy_d = v[3]; 
                            k1 = v[4]; k2 = v[5]; p1 = v[6]; p2 = v[7]; k3 = v[8];
                            foundDistortion = true;
                        }
                    }
                }

                // 4. DJI Matrix is typically Target(NIR) -> Source(CurrentBand)
                if (regex_search(xml, m, regex(R"(drone-dji:DewarpHMatrix=\"([^\"]+)\")"))) {
                    string matrixStr = m[1];
                    stringstream ss(matrixStr);
                    string segment;
                    vector<double> values;
                    while(getline(ss, segment, ',')) {
                        values.push_back(stod(segment));
                    }
                    if (values.size() == 9) {
                        for(int i=0; i<3; i++) {
                            for(int j=0; j<3; j++) {
                                H.at<double>(i, j) = values[i*3 + j];
                            }
                        }
                        foundH = true;
                    }
                }

            }
            TIFFClose(tif);
        }

        Mat img = imread(path, IMREAD_UNCHANGED | IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
        
        if (!img.empty()) {
            Mat dewarped;
            
            // --- STEP A: DISTORTION CORRECTION (PDF Page 6) ---
            if (foundDistortion && calibratedCx > 0) {
                // PDF Page 6: Camera matrix uses (CenterX + cx)
                double finalCx = calibratedCx + cx_d;
                double finalCy = calibratedCy + cy_d;
                
                Mat K = (Mat_<double>(3, 3) << fx, 0, finalCx, 0, fy, finalCy, 0, 0, 1);
                Mat D = (Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
                

                cout << "undistort: " << filename << "\n";
                undistort(img, dewarped, K, D, K);
            } else {
                dewarped = img.clone();
            }

            // --- STEP B: ALIGNMENT (PDF Page 4, Step 1) ---
            // The PDF says RelX/RelY is the offset. We construct a translation matrix.
            // Note: Direction (plus or minus) depends on if RelX is defined as (Target - Source) or vice versa.
            // Standard DJI XMP usually requires SUBTRACTING the relative offset to align to NIR.
            
            Mat aligned;

            if (foundH) {
                cout << "warpPerspective: " << filename << "\n";
                warpPerspective(dewarped, aligned, H, dewarped.size(), INTER_LINEAR | WARP_INVERSE_MAP);
            } else {
                if (abs(relX) > 0.0001 || abs(relY) > 0.0001) {
                    // Translation Matrix
                    Mat T = (Mat_<double>(2, 3) << 1, 0, relX, 0, 1, relY);
                    
                    // If the alignment is inverted, swap the signs of relX/relY below:
                    // Mat T = (Mat_<double>(2, 3) << 1, 0, -relX, 0, 1, -relY);
                    cout << "warpAffine: " << filename << "\n";
                    warpAffine(dewarped, aligned, T, dewarped.size(), INTER_LINEAR | WARP_INVERSE_MAP);
                } else {
                    aligned = dewarped;
                }    
            }

            

            // --- STEP C: OPTIONAL FINE TUNING (PDF Page 4, Step 2) ---
            // The PDF suggests OpenCV ECC here if metadata alignment isn't perfect.
            // (Not implemented in this snippet, but this is where it would go).

            imwrite(outDir + "/" + filename, aligned);
        }
    }
    return 0;
}