cmake_minimum_required(VERSION 3.10)
project(uav-plant-calibration VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Find libtiff
pkg_check_modules(TIFF REQUIRED libtiff-4)

# Common source files
set(CALIB_SOURCES
    src/calib.cc
)

# Create CLI executable (all platforms)
add_executable(calib ${CALIB_SOURCES})
target_link_libraries(calib 
    ${OpenCV_LIBS} 
    ${TIFF_LIBRARIES}
)
target_include_directories(calib PRIVATE 
    ${OpenCV_INCLUDE_DIRS} 
    ${TIFF_INCLUDE_DIRS}
)

# Windows-specific: Create GUI executable
if(WIN32)
    # GUI version with Windows support
    add_executable(calib-gui ${CALIB_SOURCES})
    target_compile_definitions(calib-gui PRIVATE WINGUI)
    target_link_libraries(calib-gui 
        ${OpenCV_LIBS} 
        ${TIFF_LIBRARIES}
        comctl32 
        gdi32 
        comdlg32 
        ole32
    )
    target_include_directories(calib-gui PRIVATE 
        ${OpenCV_INCLUDE_DIRS} 
        ${TIFF_INCLUDE_DIRS}
    )
    
    # Set as startup project in Visual Studio (optional)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT calib-gui)
endif()

# Installation rules
install(TARGETS calib DESTINATION bin)

if(WIN32)
    install(TARGETS calib-gui DESTINATION bin)
endif()

# Copy radiometric reference file to build directory
configure_file(
    ${CMAKE_SOURCE_DIR}/radiometric_reference.csv
    ${CMAKE_BINARY_DIR}/radiometric_reference.csv
    COPYONLY
)

# Print configuration summary
message(STATUS "")
message(STATUS "=== UAV Plant Calibration Configuration ===")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "Targets:")
message(STATUS "  - calib (CLI) - All platforms")
if(WIN32)
    message(STATUS "  - calib-gui (GUI) - Windows only")
endif()
message(STATUS "")
