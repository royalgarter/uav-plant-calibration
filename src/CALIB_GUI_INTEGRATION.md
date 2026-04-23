# Calib GUI Integration Guide

This document explains how to integrate the Windows GUI into `calib.cc`.

## Step 1: Add GUI Include

At the top of `calib.cc`, after the includes, add:

```cpp
// #define WINGUI  // Uncomment to enable Windows GUI

#ifdef WINGUI
#include "calib-raygui.cc"
#endif
```

## Step 2: Modify `showUsage()` Function

Update the usage function to mention the GUI option:

## Step 3: Modify `main()` Function - Add GUI Flag Handling

In the `main()` function, add GUI flag handling. Find the section where arguments are parsed and modify it:

## Step 4: Add Radiometric Reference File Loading

Before the radiometric calibration phase, add loading of the reference file from the configured path:

## Step 5: Compilation Instructions

### For Windows (with GUI):

```bash
g++ -std=c++17 -o calib-wingui.exe src/calib.cc -DWINGUI \
    -lopencv_core -lopencv_calib3d -lopencv_imgcodecs -lopencv_imgproc \
    -lopencv_video -lopencv_highgui -ltiff \
    -lraylib -lopengl32 -lgdi32 -lwinmm -lcomdlg32 -lole32 -lshell32
```

### For Linux (without GUI):

```bash
g++ -std=c++17 -o calib src/calib.cc \
    `pkg-config --cflags --libs opencv4` -ltiff
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
calib-wingui.exe --gui
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
