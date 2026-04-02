# Quick Reference - UAV Plant Calibration GUI

## Build Commands

### Windows (MinGW)
```bash
# Using build script
build-calib-gui.bat

# Manual compilation
g++ -std=c++17 -o calib-gui.exe src/calib.cc -DWINGUI ^
    -lopencv_core -lopencv_calib3d -lopencv_imgcodecs ^
    -lopencv_imgproc -lopencv_video -lopencv_highgui ^
    -ltiff -lcomctl32 -lgdi32 -lcomdlg32 -lole32 -lshell32
```

### Linux (CLI only)
```bash
g++ -std=c++17 -o calib src/calib.cc \
    `pkg-config --cflags --libs opencv4` -ltiff
```

### CMake (Cross-platform)
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Run Commands

### GUI Mode (Windows)
```bash
calib-gui.exe --gui
```

### CLI Mode
```bash
# Basic
./calib

# Full options
./calib .input .output --radio --auto 15
```

## GUI Parameters Reference

| Field | Purpose | Example Value |
|-------|---------|---------------|
| Input Folder | Raw drone images | `C:\data\drone_images` |
| Output Folder | Processed results | `C:\output\calibrated` |
| Radiometric Ref | Calibration CSV | `radiometric_reference.csv` |
| Enable Radio | Process reflectance | ☑ Checked |
| 2-Point Click | Manual board pick | ☐ Unchecked |
| Auto-Detect | Template matching | ☑ Checked |
| Board Thickness | Border pixels | `0` (auto) |

## Board Thickness Values

- **0** = Auto-detect thickness
- **-1** = Disable auto-detect (use manual 2-point click)
- **10-20** = Typical border thickness in pixels

## Workflow

```
┌─────────────────────────────────────┐
│  1. Launch GUI: calib-gui --gui     │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  2. Select Input/Output Folders     │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  3. Configure Radiometric Options   │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  4. Choose Board Detection Mode     │
│     • Auto-detect (recommended)     │
│     • Manual 2-point click          │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  5. Set Board Thickness             │
│     • 0 = auto                      │
│     • -1 = manual                   │
│     • N = specific pixels           │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  6. Click "START CALIBRATION"       │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  7. Wait for processing             │
│     • Dewarping                     │
│     • Alignment                     │
│     • Radiometric calibration       │
│     • NDVI calculation              │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  8. Check output folder             │
│     • /calib  - Aligned images      │
│     • /radio  - Radiometric imgs    │
│     • /ndvi   - NDVI maps           │
└─────────────────────────────────────┘
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| GUI won't open | Compile with `-DWINGUI` flag |
| Missing libraries | Install OpenCV, libtiff, Windows SDK |
| Board not detected | Increase thickness value or use manual mode |
| Radiometric fails | Check CSV file path and format |
| No images found | Verify input folder contains .tif/.jpg files |

## File Structure

```
uav-plant-calibration/
├── src/
│   ├── calib.cc           # Main calibration logic (MODIFIED)
│   ├── calib-gui.cc       # Windows GUI (NEW)
│   ├── GUI_README.md      # Documentation
│   └── CALIB_GUI_INTEGRATION.md
├── build-calib-gui.bat    # Windows build script (NEW)
├── CMakeLists.txt         # CMake config (NEW)
└── radiometric_reference.csv
```

## Output Files

After calibration completes:

```
output/
├── calib/           # Dewarped & aligned images
│   ├── DJI_0001_0.tif
│   ├── DJI_0001_1.tif
│   └── ...
├── radio/           # Radiometrically calibrated
│   ├── DJI_0001_0.tif
│   └── ...
├── ndvi/            # NDVI maps
│   ├── DJI_0001_NDVI_raw.tif
│   └── DJI_0001_NDVI_color.jpg
└── radiometric_report.csv  # DN values & coefficients
```
