# UAV Calibration command

## Install

**install opencv**

### For linux

```
git clone https://github.com/opencv/opencv
mkdir opencv/build
cd opencv/build
cmake ..
sudo make install
```

OR

```
npm run install:opencv
```

### For mac

```
brew tap homebrew/science
brew install opencv@3
brew link --force opencv@3
```

### For window

Required:
- [MinGW-w64 for Windows](https://winlibs.com/), [Direct](https://github.com/brechtsanders/winlibs_mingw/releases/download/15.2.0posix-13.0.0-msvcrt-r1/winlibs-x86_64-posix-seh-gcc-15.2.0-mingw-w64msvcrt-13.0.0-r1.zip)
- [OpenCV-4.5.5-x64 | MinGW-Build](https://github.com/huihut/OpenCV-MinGW-Build/tree/master), [Direct](https://github.com/huihut/OpenCV-MinGW-Build/archive/refs/tags/OpenCV-4.5.5-x64.zip)
- Libtiff in `libtiff.zip` => Extract to libtiff folder

#### Environment:

```
set PATH=%PATH%;c:\mingw64\bin\;c:\opencv\x64\mingw\bin
set OPENCV_DIR=c:\opencv\
```

In window_build includes
- Required MinGW DLL (in case moving executable to other machine):
- Required OpenCV DLL (in case moving executable to other machine):
- Required Libtiff DLL (in case moving executable to other machine):


## Usage

See all scripts in `package.json`.
All window scripts are compatible with Window Power Shell
