@echo off
SETLOCAL
REM DEPRECATED: launches the native C (mongoose) web server, which is no longer used.
REM Use calib-web-node.bat instead (runs webui\server.js on the NodeJS binary).
REM Move to the script's directory to ensure relative paths work correctly
pushd "%~dp0"
REM Execute the calibration tool from the window_build directory
"%~dp0window_build\calib-web.exe"
popd
ENDLOCAL

