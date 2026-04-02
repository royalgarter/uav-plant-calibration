# Calib GUI Integration Guide

This document explains how to integrate the Windows GUI into `calib.cc`.

## Step 1: Add GUI Include

At the top of `calib.cc`, after the includes, add:

```cpp
// #define WINGUI  // Uncomment to enable Windows GUI

#ifdef WINGUI
#include "calib-gui.cc"
#endif
```

## Step 2: Modify `showUsage()` Function

Update the usage function to mention the GUI option:

```cpp
void showUsage() {
    cout << "USAGE: ./calib <src_dir (default: .input/)> <dest_dir (default: .output/)> [--radio] [--auto]" << endl;
    cout << "  --radio       Enable radiometric calibration." << endl;
    cout << "  --auto        Auto-detect radiometric board (used with --radio). Optional: --auto <border_thickness>" << endl;
#ifdef WINGUI
    cout << "  --gui         Launch Windows GUI interface." << endl;
#endif
    cout << "" << endl;
    cout << "---" << endl;
    cout << "" << endl;
}
```

## Step 3: Modify `main()` Function - Add GUI Flag Handling

In the `main()` function, add GUI flag handling. Find the section where arguments are parsed and modify it:

```cpp
int main(int argc, char** argv) {
    // Generate logfilename
    auto t = time(nullptr);
    auto tm = *localtime(&t);
    ostringstream oss;
    oss << put_time(&tm, "%y%m%d-%H%M") << ".log";
    gLog.open(oss.str());

    string inDir = ".input";
    string outDir = ".output";
    string radioRefFile = "radiometric_reference.csv";
    bool doRadio = false;
    int autoRadioThickness = -1;
    Point radioInterval(40, 0);
    bool useGui = false;

    // Check for GUI flag first
    if (argc > 1 && string(argv[1]) == "--gui") {
        useGui = true;
    }

    if (useGui) {
        #ifdef WINGUI
        cout << "Launching GUI..." << endl;
        
        bool guiTwoPointClick, guiAutoDetect;
        int guiBoardThickness;
        
        if (!runCalibGui(inDir, outDir, radioRefFile, doRadio, guiTwoPointClick, guiAutoDetect, guiBoardThickness)) {
            cout << "GUI cancelled or exited." << endl;
            return 0;
        }
        
        // Apply GUI settings
        if (guiAutoDetect) {
            autoRadioThickness = (guiBoardThickness == 0) ? 0 : guiBoardThickness;
        }
        if (guiTwoPointClick) {
            autoRadioThickness = -1; // Disable auto-detect, use manual 2-point click
        }
        
        cout << "GUI Configuration:" << endl;
        cout << "  Input Folder: " << inDir << endl;
        cout << "  Output Folder: " << outDir << endl;
        cout << "  Radiometric Ref: " << radioRefFile << endl;
        cout << "  Radiometric Calibration: " << (doRadio ? "ENABLED" : "disabled") << endl;
        cout << "  Auto-Detect Board: " << (guiAutoDetect ? "ENABLED" : "disabled") << endl;
        if (guiAutoDetect) {
            cout << "  Board Thickness: " << autoRadioThickness << endl;
        }
        cout << endl;
        
        #else
        cerr << "Error: GUI support not compiled. Define WINGUI and compile with MinGW-w64." << endl;
        return 1;
        #endif
    } else {
        // Original CLI argument parsing
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
                autoRadioThickness = 0;
                if (i + 1 < argc && isdigit(argv[i+1][0])) {
                    autoRadioThickness = stoi(argv[i+1]);
                    i++;
                }
            } else {
                args.push_back(arg);
            }
        }

        if (args.size() > 0) inDir = args[0];
        if (args.size() > 1) outDir = args[1];
    }

    // ... rest of the code continues unchanged ...
```

## Step 4: Add Radiometric Reference File Loading

Before the radiometric calibration phase, add loading of the reference file from the configured path:

Find this section in the radiometric calibration phase:

```cpp
if (doRadio) {
    gLog << "\n--- RADIOMETRIC CALIBRATION PHASE ---" << endl;

    // Load radiometric reference data from config file
    loadRadiometricRefs("radiometric_reference.csv");  // <-- Replace this line
```

Replace the hardcoded filename with the variable:

```cpp
if (doRadio) {
    gLog << "\n--- RADIOMETRIC CALIBRATION PHASE ---" << endl;

    // Load radiometric reference data from config file
    loadRadiometricRefs(radioRefFile);
```

## Step 5: Compilation Instructions

### For Windows (with GUI):

```bash
g++ -std=c++17 -o calib.exe src/calib.cc -DWINGUI \
    -lopencv_core -lopencv_calib3d -lopencv_imgcodecs -lopencv_imgproc \
    -lopencv_video -lopencv_highgui -ltiff \
    -lcomctl32 -lgdi32 -lcomdlg32 -lole32
```

### For Linux (without GUI):

```bash
g++ -std=c++17 -o calib src/calib.cc \
    `pkg-config --cflags --libs opencv4` -ltiff
```

### Using CMake (Recommended):

Create or update `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(uav-plant-calibration)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TIFF REQUIRED libtiff-4)

# CLI version (Linux/Windows)
add_executable(calib src/calib.cc)
target_link_libraries(calib ${OpenCV_LIBS} ${TIFF_LIBRARIES})
target_include_directories(calib PRIVATE ${OpenCV_INCLUDE_DIRS} ${TIFF_INCLUDE_DIRS})

# Windows GUI version
if(WIN32)
    add_executable(calib-gui src/calib.cc)
    target_compile_definitions(calib-gui PRIVATE WINGUI)
    target_link_libraries(calib-gui ${OpenCV_LIBS} ${TIFF_LIBRARIES}
                          comctl32 gdi32 comdlg32 ole32)
    target_include_directories(calib-gui PRIVATE ${OpenCV_INCLUDE_DIRS} ${TIFF_INCLUDE_DIRS})
endif()
```

## GUI Features

The GUI provides the following input options:

1. **Input Folder** - Browse button to select the input directory containing raw images
2. **Output Folder** - Browse button to select the output directory for processed images
3. **Radiometric Reference File** - Browse button to select the CSV file with radiometric reference data
4. **Enable Radiometric Calibration** - Checkbox to enable `--radio` option
5. **2-Point Click Mode** - Checkbox for manual board detection (click 56% and 3% patches)
6. **Auto-Detect Board** - Checkbox for automatic board detection using template matching
7. **Board Thickness** - Numeric input for border thickness (0 = auto-detect, -1 = disable)

## Usage Examples

### Launch GUI (Windows):
```bash
calib-gui.exe --gui
```

### CLI Mode (unchanged):
```bash
# Basic usage
./calib .input .output

# With radiometric calibration
./calib .input .output --radio

# With auto-detect board
./calib .input .output --radio --auto 15
```

## Notes

- The GUI is only available on Windows (requires `WINGUI` preprocessor definition)
- When both "2-Point Click Mode" and "Auto-Detect Board" are toggled, they mutually exclude each other
- Board thickness value meanings:
  - `0` = Auto-detect the thickness
  - `-1` = Disable auto-detection (use manual 2-point click)
  - `>0` = Use specific pixel thickness value
