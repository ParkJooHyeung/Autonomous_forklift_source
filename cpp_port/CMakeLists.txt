cmake_minimum_required(VERSION 3.10)
project(AutonomousForklift CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add submodule include
include_directories(lib/json/include)

find_package(Eigen3 CONFIG REQUIRED)
if (Eigen3_FOUND)
    include_directories(${Eigen3_INCLUDE_DIR})
    message(STATUS "Found Eigen3: ${Eigen3_INCLUDE_DIR}")
else ()
    message(FATAL_ERROR "Eigen3 not found!")
endif ()

# Find RealSense library
find_package(realsense2 REQUIRED)
if (realsense2_FOUND)
    include_directories(${realsense2_INCLUDE_DIRS})
    message(STATUS "Found RealSense: ${realsense2_INCLUDE_DIRS}")
    # Explicitly add Homebrew include path for librealsense2 headers if needed
    include_directories(/opt/homebrew/include)
else ()
    message(FATAL_ERROR "RealSense library not found!")
endif ()

# Find OpenCV library
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS}")
else ()
    message(FATAL_ERROR "OpenCV library not found!")
endif ()


# Add include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/BaseController.cpp
    src/calculations.cpp
    src/KalmanFilterWrapper.cpp
    src/RealSenseManager.cpp
    src/YoloDetector.cpp
    src/ForkliftController.cpp
    src/PalletTracker.cpp
    src/PalletForker.cpp
)

# Executable
add_executable(AutonomousForklift ${SOURCES})

target_link_libraries(AutonomousForklift Eigen3::Eigen /opt/homebrew/lib/librealsense2.dylib ${OpenCV_LIBS})

# Optionally, specify output directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)