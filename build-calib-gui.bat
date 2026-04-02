@echo off
REM Build script for UAV Plant Calibration with Windows GUI
REM Requires: MinGW-w64 with g++ in PATH, OpenCV, and libtiff

echo ========================================
echo UAV Plant Calibration - Build Script
echo ========================================
echo.

REM Check if g++ is available
where g++ >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: g++ not found in PATH!
    echo Please install MinGW-w64 and add it to your PATH.
    echo Download from: https://www.mingw-w64.org/
    pause
    exit /b 1
)

echo [1/3] Checking dependencies...

REM Check for OpenCV (adjust OPENCV_DIR if needed)
if not defined OPENCV_DIR (
    echo WARNING: OPENCV_DIR not set. Using default paths...
    set OPENCV_DIR=C:\opencv
)

if not exist "%OPENCV_DIR%" (
    echo WARNING: OpenCV not found at %OPENCV_DIR%
    echo Please set OPENCV_DIR environment variable
)

echo   - OpenCV Dir: %OPENCV_DIR%
echo   - Compiler: g++
echo.

echo [2/3] Compiling calib.exe (CLI version)...
g++ -std=c++17 -o calib.exe src/calib.cc ^
    -lopencv_core -lopencv_calib3d -lopencv_imgcodecs ^
    -lopencv_imgproc -lopencv_video -lopencv_highgui ^
    -ltiff

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: CLI compilation failed!
    echo Make sure OpenCV and libtiff libraries are installed.
    pause
    exit /b 1
)
echo   SUCCESS: calib.exe created
echo.

echo [3/3] Compiling calib-gui.exe (with Windows GUI)...
g++ -std=c++17 -o calib-gui.exe src/calib.cc -DWINGUI ^
    -lopencv_core -lopencv_calib3d -lopencv_imgcodecs ^
    -lopencv_imgproc -lopencv_video -lopencv_highgui ^
    -ltiff -lcomctl32 -lgdi32 -lcomdlg32 -lole32

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: GUI compilation failed!
    echo Make sure Windows SDK libraries are available.
    pause
    exit /b 1
)
echo   SUCCESS: calib-gui.exe created
echo.

echo ========================================
echo Build completed successfully!
echo ========================================
echo.
echo Usage:
echo   CLI Mode:  calib.exe [input_dir] [output_dir] [--radio] [--auto]
echo   GUI Mode:  calib-gui.exe --gui
echo.
echo Files created:
echo   - calib.exe        (Command-line interface)
echo   - calib-gui.exe    (Windows GUI interface)
echo.
pause
