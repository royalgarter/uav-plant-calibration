cmake_minimum_required(VERSION 3.10)
project(FisheyeCLI CXX)

# Find OpenCV
# For Windows, you might need to set OpenCV_DIR environment variable
# to the build directory of OpenCV (e.g., C:\opencv\build)
find_package(OpenCV 4.5.5 REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "OpenCV 4.5.5 found.")
    message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV 4.5.5 not found. Please set OpenCV_DIR.")
endif()

# Add executable
add_executable(fisheye src/cli.cc)

# Link OpenCV libraries
target_link_libraries(fisheye ${OpenCV_LIBS} user32 gdi32 comctl32)

# Set C++ standard
set_property(TARGET fisheye PROPERTY CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

# Configure as a Windows GUI application
# set_property(TARGET fisheye PROPERTY WIN32_EXECUTABLE TRUE)
add_definitions(-D_WIN32_WINNT=0x0601) # Target Windows 7 or later for InitCommonControlsEx

set(BUILD_SHARED_LIBS_DEFAULT OFF)