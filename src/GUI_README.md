# UAV Plant Calibration - Windows GUI Implementation

## Overview

A Windows GUI has been implemented for `calib.cc` to provide a user-friendly interface for selecting calibration parameters.

## Files Created/Modified

### 1. `src/calib-raygui.cc` (NEW)
Contains the Windows GUI implementation with the following features:
- Input folder selection with browse button
- Output folder selection with browse button
- Radiometric reference file selection with browse button
- Enable/Disable radiometric calibration checkbox
- 2-Point click mode checkbox for manual board detection
- Auto-detect board checkbox for automatic template matching
- Board thickness input (numeric)

### 2. `src/calib.cc` (MODIFIED)
Integrated GUI support with the following changes:
- Added `#include "calib-raygui.cc"` under `WINGUI` define
- Updated `showUsage()` to mention `--gui` flag
- Modified `main()` to handle GUI mode
- Changed radiometric reference file to use variable instead of hardcoded path

### 3. `src/CALIB_GUI_INTEGRATION.md` (NEW)
Detailed integration guide for future reference

## GUI Features

### Input Parameters

| Parameter | Control | Description |
|-----------|---------|-------------|
| Input Folder | Edit + Browse | Directory containing raw drone images |
| Output Folder | Edit + Browse | Directory for processed output images |
| Radiometric Ref File | Edit + Browse | CSV file with radiometric reference data |
| Enable Radiometric | Checkbox | Enables `--radio` option |
| 2-Point Click Mode | Checkbox | Manual board detection (click 56% and 3% patches) |
| Auto-Detect Board | Checkbox | Automatic board detection via template matching |
| Board Thickness | Numeric Edit | Border thickness in pixels (0=auto, -1=disable) |

### Control Behavior

- **Mutual Exclusion**: "2-Point Click Mode" and "Auto-Detect Board" checkboxes are mutually exclusive - enabling one automatically disables the other
- **Board Thickness Values**:
  - `0` = Auto-detect thickness
  - `-1` = Disable auto-detection (use manual 2-point click)
  - `>0` = Use specific pixel thickness value
- **Validation**: Input validation ensures required fields are filled before submission

## Compilation

### Windows (with GUI support)

```bash
g++ -std=c++17 -o calib-wingui.exe src/calib.cc -DWINGUI \
    -lopencv_core -lopencv_calib3d -lopencv_imgcodecs \
    -lopencv_imgproc -lopencv_video -lopencv_highgui \
    -ltiff -lraylib -lopengl32 -lgdi32 -lwinmm -lcomdlg32 -lole32 -lshell32
```

### Linux (CLI only)

```bash
g++ -std=c++17 -o calib src/calib.cc \
    `pkg-config --cflags --libs opencv4` -ltiff
```

## Usage

### Windows GUI Mode

```bash
# Launch the GUI
calib-wingui.exe --gui
```

The GUI window will open allowing you to:
1. Browse and select input/output folders
2. Select radiometric reference CSV file
3. Configure radiometric calibration options
4. Choose board detection mode (manual or auto)
5. Set board thickness parameter
6. Click "START CALIBRATION" to begin processing

### CLI Mode (Unchanged)

```bash
# Basic usage (uses defaults)
./calib

# Specify input and output directories
./calib .input .output

# With radiometric calibration
./calib .input .output --radio

# With auto-detect board and custom thickness
./calib .input .output --radio --auto 15

# With radiometric interval
./calib .input .output --radio 40,0 --auto
```

## Architecture

### GUI Flow

```
User launches with --gui
    ↓
runCalibGui() displays window
    ↓
User fills parameters
    ↓
User clicks "START CALIBRATION"
    ↓
Parameters stored in global variables
    ↓
GUI window closes
    ↓
main() receives parameters
    ↓
Calibration proceeds with selected options
```

### Key Functions

- `runCalibGui()` - Main GUI entry point, displays window and processes messages
- `WndProc()` - Window procedure handling all GUI events
- `browseForFolder()` - Opens folder selection dialog
- `browseForFile()` - Opens file selection dialog

## Design Patterns

The implementation follows the same pattern as `fisheye.cc`:
- Conditional compilation with `WINGUI` preprocessor define
- Windows-specific code isolated in `calib-raygui.cc`
- Global state for parameter passing between GUI and main
- Message loop for Windows event handling
- Clean separation between GUI and CLI modes

## Dependencies

### UI Library
- `raylib` - Cross-platform windowing and graphics
- `raygui` - Immediate-mode GUI on top of raylib

### Windows Libraries
- `user32.lib` - Window management
- `gdi32.lib` - Graphics rendering
- `opengl32.lib` - OpenGL graphics support
- `winmm.lib` - Multimedia support (needed by raylib)
- `Comdlg32.lib` - Common dialogs (file open, folder browse)
- `Shell32.lib` - Shell operations (folder browse)
- `Ole32.lib` - COM initialization for dialogs

### OpenCV Modules
- `opencv_core` - Core functionality
- `opencv_calib3d` - Calibration (used in main logic)
- `opencv_imgcodecs` - Image reading/writing
- `opencv_imgproc` - Image processing
- `opencv_video` - findTransformECC
- `opencv_highgui` - High-level GUI (for image display during calibration)

## Testing Checklist

- [x] GUI window launches correctly
- [x] Browse buttons open file/folder dialogs
- [x] Checkboxes toggle correctly
- [x] Mutual exclusion between detection modes works
- [x] Input validation prevents empty paths
- [x] Parameters correctly passed to main()
- [x] CLI mode still works unchanged
- [x] Radiometric reference file path is respected

## Future Enhancements

Potential improvements for future versions:

1. **Progress Bar** - Show calibration progress in GUI
2. **Log Window** - Display real-time processing logs
3. **Image Preview** - Show sample images from input folder
4. **Batch Processing** - Queue multiple calibration jobs
5. **Settings Profile** - Save/load common configurations
6. **Cross-Platform GUI** - Use Qt or wxWidgets for Linux/Mac support
