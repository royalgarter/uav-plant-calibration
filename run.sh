#!/bin/sh

# Use gcc to compile C++ files
CC="gcc"
MINGW="x86_64-w64-mingw32-gcc"
CFLAGS="-std=c++17"
# Link the C++ standard library at the end
LIBS="-lstdc++"

# Helper for cross-platform commands
case "$(uname -s)" in
    MSYS*|MINGW*|CYGWIN*)
        PS_CMD="powershell.exe"
        ;;
    *)
        PS_CMD="pwsh" # or powershell if installed on Linux
        ;;
esac

run_cmd() {
    echo "EXECUTING:"
    echo "$@"
    echo ""
    "$@"
}

case "$1" in
    "build:dev")
        run_cmd node-gyp -j 8 rebuild --debug
        ;;
    "build")
        run_cmd node-gyp -j 8 rebuild
        ;;
    "build:calib")
        run_cmd $CC $CFLAGS src/calib.cc -o calib -ltiff $(pkg-config --cflags --libs opencv4) $LIBS
        ;;
    "build:calib-mingw")
        run_cmd $MINGW $CFLAGS src/calib.cc -o calib -ltiff -Ilibtiff -Ilibtiff/include -Llibtiff/lib $(pkg-config --cflags --libs opencv4) $LIBS
        ;;
    "build:calib-win")
        run_cmd $CC $CFLAGS src/calib.cc -o window_build/calib -ltiff -Ilibtiff -Ilibtiff/include -Llibtiff/lib -Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 $LIBS
        ;;
    "example:calib")
        run_cmd ./calib example/calib/input example/calib/output
        ;;
    "build:cli")
        run_cmd $CC $CFLAGS src/cli.cc -o fisheye $(node utils/find-opencv.js --cflags) $(node utils/find-opencv.js --libs) $LIBS
        ;;
    "build:cli-win")
        run_cmd $CC $CFLAGS src/cli.cc -o window_build/fisheye -Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 $LIBS
        ;;
    "example:cli")
        run_cmd ./fisheye example/fisheye/input example/fisheye/output example/fisheye/checkboard 9 6
        ;;
    "example:cli-win:calibrate")
        run_cmd ./window_build/fisheye.exe example/fisheye/input example/fisheye/output example/fisheye/checkboard 9 6
        ;;
    "example:cli-win:export")
        run_cmd ./window_build/fisheye.exe example/fisheye/input example/fisheye/output example/fisheye/checkboard/calibration.txt 9 6
        ;;
    "example:cli-win:import")
        run_cmd ./window_build/fisheye.exe example/fisheye/input example/fisheye/output example/fisheye/checkboard/calibration.txt
        ;;
    "cmake:cli-win")
        run_cmd cmake --build . --target clean && run_cmd cmake . && run_cmd cmake --build .
        ;;
    "clean")
        run_cmd node-gyp clean
        ;;
    "release:win")
        run_cmd $PS_CMD -Command "Compress-Archive -Path window_build/ -DestinationPath release_$(date +'%Y%m%d_%H%M').zip -Force"
        ;;
    "example:js")
        run_cmd node index.js example/input/IMG-0.jpg out_js.jpg example/checkboard 9 6
        ;;
    "install:opencv")
        run_cmd sudo apt update && run_cmd sudo apt install -y python3 python3-pip python3-dev python3-venv build-essential libtiff6 libopencv-dev python3-opencv
        ;;
    "start")
        run_cmd node index.js
        ;;
    *)
        echo "Usage: $0 {command}"
        echo ""
        echo "Available commands:"
        grep -E '^\s*".*"\)' $0 | sed 's/^[[:space:]]*"\([^"]*\)").*/  \1/'
        echo ""
        echo "Example: $0 build:calib"
        exit 1
        ;;
esac
