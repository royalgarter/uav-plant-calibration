#!/bin/bash

CC="gcc"
CFLAGS="-std=c++17 -fopenmp"
CW_FLAGS="-Wall -Wextra -pedantic"

LIBS="-lstdc++"
TIFFLIB="-ltiff -Ilibtiff -Ilibtiff/include -Llibtiff/lib"
OPENCVLIB_WIN="-Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 -lopencv_highgui455 -lopencv_features2d455"
RAYLIB="-Iraylib/include -Lraylib/lib -lraylib"
WINGUILIB="-DWINGUI -lopengl32 -lgdi32 -lwinmm -lcomdlg32 -lcomctl32 -lole32 -lshell32 -luuid"
RELEASE_FILES="calib-web-gui.bat calib-window-gui.bat window_build/ .input_ref/ webui/ package.json"
RELEASE_FILES_WIN=${RELEASE_FILES// /\,}
RELEASE_TIME="$(date +'%Y%m%d_%H%M')"

echo ""
if [[ "$1" == *"win"* ]]; then
    echo "OS: window"
    PS_CMD="powershell.exe -Command"
else
    echo "OS: linux"
    PS_CMD=""
fi

run_cmd() {
    echo "---"
    echo "$@"
    echo "---"
    $PS_CMD "$@"
}

case "$1" in
    "build:all")
        ./run.sh build:calib;
        ./run.sh build:calib-win;
        ./run.sh build:calib-win-gui;
        ./run.sh build:calib-webui;
        ./run.sh build:calib-webui-win;
        ;;

    ###############################################################################################

    "build:calib")
        run_cmd $CC $CFLAGS src/calib.cc src/calib-func.cc -o calib $LIBS -ltiff $(pkg-config --cflags --libs opencv4)
        find . -type f -name "calib" -executable
        ;;
    "build:calib-webui")
        run_cmd $CC $CFLAGS src/calib-web.cc src/mongoose.c src/cJSON.c -o calib-web $LIBS
        find . -type f -name "calib-web" -executable
        ;;
    
    ###############################################################################################        

    "build:calib-win")
        run_cmd $CC $CFLAGS src/calib.cc src/calib-func.cc -o window_build/calib.exe $LIBS $TIFFLIB $OPENCVLIB_WIN
        find . -type f -name "calib.exe"
        ;;
    "build:calib-webui-win")
        run_cmd $CC $CFLAGS src/calib-web.cc src/mongoose.c src/cJSON.c -o window_build/calib-web.exe $LIBS -lws2_32
        find . -type f -name "calib-web.exe"
        ;;
    "build:calib-win-gui")
        run_cmd $CC $CFLAGS src/calib.cc src/calib-func.cc -o window_build/calib-wingui.exe $LIBS $TIFFLIB $OPENCVLIB_WIN $RAYLIB $WINGUILIB
        find . -type f -name "calib-wingui.exe"
        ;;
    
    ###############################################################################################
    
    "example:calib")
        run_cmd ./calib .input .output --optimize 
        ;;
    "example:calib-radio")
        run_cmd ./calib .input .output --optimize --radio 0,25
        ;;
    "example:calib-auto")
        run_cmd ./calib .input .output --optimize --radio --auto 10 --ref .input_ref/radiometric_reference.csv --template .input_ref/radiometric_board.jpg
        ;;
    
    ###############################################################################################

    "example:calib-win")
        run_cmd ./window_build/calib-wingui.exe .input .output --optimize
        ;;
    "example:calib-win-radio")
        run_cmd ./window_build/calib-wingui.exe .input .output --optimize --radio 0,25
        ;;
    "example:calib-win-auto")
        run_cmd ./window_build/calib-wingui.exe .input .output --optimize --radio 0,25 --auto 10
        ;;
    "example:calib-win-gui")
        run_cmd ./window_build/calib-wingui.exe --gui
        ;;
    
    ############################################################################################### 
    
    "build:fisheye")
        run_cmd $CC $CFLAGS src/cli.cc -o fisheye $(pkg-config --cflags --libs opencv4) $LIBS
        ;;
    "build:fisheye-win")
        run_cmd $CC $CFLAGS src/cli.cc -o window_build/fisheye -Ic:/opencv -Ic:/opencv/include -Lc:/opencv/x64/mingw/lib/ -lopencv_core455 -lopencv_calib3d455 -lopencv_imgcodecs455 -lopencv_imgproc455 -lopencv_video455 -lopencv_highgui455 $LIBS
        ;;
    
    ###############################################################################################
    
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
    
    ###############################################################################################
    
    "release:zip")
        run_cmd zip -r release_$RELEASE_TIME.zip $RELEASE_FILES
        rm -rf .tmp
            mkdir -p .tmp/.input .tmp/.output
            pushd .tmp
            zip -r ../release_$RELEASE_TIME.zip .input .output
            popd
        rm -rf .tmp
        ;;
    "release:win")
        run_cmd Compress-Archive -Path $RELEASE_FILES_WIN -DestinationPath release_$RELEASE_TIME.zip -Force -Verbose
        ;;
    
    ###############################################################################################        
    
    "install:opencv")
        run_cmd sudo apt update && run_cmd sudo apt install -y python3 python3-pip python3-dev python3-venv build-essential libtiff6 libopencv-dev python3-opencv libtbb-dev
        ;;

    ###############################################################################################
    
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
