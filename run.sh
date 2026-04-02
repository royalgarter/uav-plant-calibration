#!/bin/bash

# Use gcc to compile C++ files
CC="gcc"
MINGW="x86_64-w64-mingw32-gcc"
CFLAGS="-Wall -Wextra -pedantic -std=c++17"
# Link the C++ standard library at the end
LIBS="-lstdc++"
WINGUI="-DWINGUI -lcomctl32 -lgdi32 -lcomdlg32 -lole32 -lshell32"

if [[ "$1" == *"win"* ]]; then
    echo "OS: window"
    PS_CMD="powershell.exe -Command"
else
    echo "OS: linux"
    PS_CMD=""
    
fi

run_cmd() {
    echo "$@"
    echo "---"
    $PS_CMD "$@"
}

case "$1" in
    "build:calib")
        run_cmd $CC $CFLAGS src/calib.cc -o calib -ltiff $(pkg-config --cflags --libs opencv4) $LIBS
        ;;
    "build:calib-mingw")
        run_cmd $MINGW $CFLAGS src/calib.cc -o calib -ltiff -Ilibtiff -Ilibtiff/include -Llibtiff/lib $(pkg-config --cflags --libs opencv4) $LIBS
        ;;
    "build:calib-win")
        run_cmd $CC $CFLAGS src/calib.cc -o window_build/calib.exe -ltiff -Ilibtiff -Ilibtiff/include -Llibtiff/lib -Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 -lopencv_highgui455 $LIBS
        ;;
    "build:calib-win-gui")
        run_cmd $CC $CFLAGS src/calib.cc -o window_build/calib-gui.exe -ltiff -Ilibtiff -Ilibtiff/include -Llibtiff/lib -Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 -lopencv_highgui455 $LIBS $WINGUI
        ;;
    "example:calib")
        run_cmd ./calib example/calib/input example/calib/output
        ;;
    "example:calib-radio")
        run_cmd ./calib example/calib/input example/calib/output --radio 0,25
        ;;
    "example:calib-auto")
        run_cmd ./calib example/calib/input example/calib/output --radio 0,25 --auto 10 --template example/calib/radiometric_board.jpg
        ;;
    "example:calib-win")
        run_cmd ./window_build/calib.exe ./example/calib/input/ ./example/calib/output/
        ;;
    "example:calib-win-radio")
        run_cmd ./window_build/calib.exe ./example/calib/input/ ./example/calib/output/ --radio 0,25
        ;;
    "example:calib-win-auto")
        run_cmd ./window_build/calib.exe ./example/calib/input/ ./example/calib/output/ --radio 0,25 --auto 10
        ;;
    "build:fisheye")
        run_cmd $CC $CFLAGS src/cli.cc -o fisheye $(pkg-config --cflags --libs opencv4) $LIBS
        ;;
    "build:fisheye-win")
        run_cmd $CC $CFLAGS src/cli.cc -o window_build/fisheye -Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 -lopencv_highgui455 $LIBS
        ;;
    "example:fisheye")
        run_cmd ./fisheye example/fisheye/input example/fisheye/output example/fisheye/checkboard 9 6
        ;;
    "example:fisheye-win:calibrate")
        run_cmd ./window_build/fisheye.exe example/fisheye/input example/fisheye/output example/fisheye/checkboard 9 6
        ;;
    "example:fisheye-win:export")
        run_cmd ./window_build/fisheye.exe example/fisheye/input example/fisheye/output example/fisheye/checkboard/calibration.txt 9 6
        ;;
    "example:fisheye-win:import")
        run_cmd ./window_build/fisheye.exe example/fisheye/input example/fisheye/output example/fisheye/checkboard/calibration.txt
        ;;
    "release:zip")
        run_cmd zip -r release_$(date +'%Y%m%d_%H%M').zip calib-window.bat calib-window-gui.bat window_build/ 
        ;;
    "release:win")
        run_cmd Compress-Archive -Path calib-window.bat, calib-window-gui.bat, window_build/ -DestinationPath release_$(date +'%Y%m%d_%H%M').zip -Force -Verbose
        ;;
    "install:opencv")
        run_cmd sudo apt update && run_cmd sudo apt install -y python3 python3-pip python3-dev python3-venv build-essential libtiff6 libopencv-dev python3-opencv libtbb-dev
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
