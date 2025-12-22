# Fisheye camera model

A opencv fisheye camera model bindings for Node.js.


## Install

**install opencv 3.x**

### For linux

```
git clone https://github.com/opencv/opencv
mkdir opencv/build
cd opencv/build
cmake ..
sudo make install
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
- [CMake - cmake-4.2.1-windows-x86_64.msi](https://cmake.org/download/), [Direct](https://github.com/Kitware/CMake/releases/download/v4.2.1/cmake-4.2.1-windows-x86_64.msi)
- [OpenCV-4.5.5-x64 | MinGW-Build](https://github.com/huihut/OpenCV-MinGW-Build/tree/master), [Direct](https://github.com/huihut/OpenCV-MinGW-Build/archive/refs/tags/OpenCV-4.5.5-x64.zip)


Environment:

```
set PATH=%PATH%;c:\mingw64\bin\;C:\Program Files\CMake\bin;c:\opencv\x64\mingw\bin
set OPENCV_DIR=c:\opencv\
```

Required OpenCV DLL (in case moving executable to other machine):
- libopencv_calib3d455.dll
- libopencv_core455.dll
- libopencv_features2d455.dll
- libopencv_flann455.dll
- libopencv_imgcodecs455.dll
- libopencv_imgproc455.dll

```
npm run build:cli-win
npm run example:cli-win
```

**install npm package**

```
npm install @sigodenjs/fisheye
```

## Usage

### Prepare checkboard

Download the [checkerboard pattern](https://github.com/sigoden/node-fisheye/blob/master/doc/checkboard.webp?raw=true) and print it on a paper (letter or A4 size). You also want to attach the paper to a hard, flat surface such as a piece of cardboard. The key here: straight lines need to be straight.

### Take sample photos

Hold the pattern in front of your camera and capture some images. You want to hold the pattern in different positions and angles. The key here: the patterns need to appear distorted in a different ways (so that OpenCV knows as much about your lens as possible). 

![demo](https://raw.githubusercontent.com/sigoden/node-fisheye/master/doc/sample.png) 

### Find K and D

```js
let imgs = fs
    .readdirSync('example/samples')
    .map(file => fs.readFileSync('example/samples/'+file));
let {K, D} = fisheye.calibrate(imgs, 9, 6);
```

### Undistort image

```js
let img = fs.readFileSync('example/samples/IMG-0.jpg');
let buf = fisheye.undistort(img, K, D);
fs.writeFileSync('/tmp/IMG-0.jpg', buf);
```

![before](https://raw.githubusercontent.com/sigoden/node-fisheye/master/example/samples/IMG-0.jpg) --> ![after](https://raw.githubusercontent.com/sigoden/node-fisheye/master/doc/IMG-0.jpg)

## License

Copyright (c) 2018 sigoden

Licensed under the MIT license.