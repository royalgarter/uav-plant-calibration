@echo off
SETLOCAL
REM Move to the script's directory to ensure relative paths work correctly
pushd "%~dp0"
REM Execute the calibration tool from the window_build directory
"%~dp0window_build\calib-wingui.exe" --gui%*
popd
ENDLOCAL
