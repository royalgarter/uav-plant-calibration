::[Bat To Exe Converter]
::
::YAwzoRdxOk+EWAjk
::fBw5plQjdDPlD0qIuk4/KxpYRAuRKFePBKAS/P3H5+WKp3EPW/s+dIjayKCLMtwU41HYYJc7m2pWmcUYQRhZchupfA4goGFMinOANdKVjwngXEGK6UV+EmZ75w==
::YAwzuBVtJxjWCl3EqQJgSA==
::ZR4luwNxJguZRRnk
::Yhs/ulQjdF+5
::cxAkpRVqdFKZSDk=
::cBs/ulQjdF+5
::ZR41oxFsdFKZSDk=
::eBoioBt6dFKZSDk=
::cRo6pxp7LAbNWATEpCI=
::egkzugNsPRvcWATEpCI=
::dAsiuh18IRvcCxnZtBJQ
::cRYluBh/LU+EWAnk
::YxY4rhs+aU+JeA==
::cxY6rQJ7JhzQF1fEqQJQ
::ZQ05rAF9IBncCkqN+0xwdVs0
::ZQ05rAF9IAHYFVzEqQJQ
::eg0/rx1wNQPfEVWB+kM9LVsJDGQ=
::fBEirQZwNQPfEVWB+kM9LVsJDGQ=
::cRolqwZ3JBvQF1fEqQJQ
::dhA7uBVwLU+EWDk=
::YQ03rBFzNR3SWATElA==
::dhAmsQZ3MwfNWATElA==
::ZQ0/vhVqMQ3MEVWAtB9wSA==
::Zg8zqx1/OA3MEVWAtB9wSA==
::dhA7pRFwIByZRRnk
::Zh4grVQjdDPlD0qIuk4/KxpYRAuRKFePBKAS/P3H5+WKp3EPW/s+dIjayKCLMtwU41HYYJc7m2pWmcUYQRhZchupfA4goGFMimeEO86e/hjkSAaM/k5Q
::YB416Ek+ZG8=
::
::
::978f952a14a936cc963da21a135fa983
@echo off
SETLOCAL
REM Move to the script's directory to ensure relative paths work correctly
pushd "%~dp0"
REM Execute the calibration tool from the window_build directory
"%~dp0window_build\node.exe" "webui\server.js"
popd
ENDLOCAL

