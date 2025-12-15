# Autonomous Forklift Control System

## Overview

This project contains the source code for an autonomous forklift control system. It uses computer vision (YOLOv5) and an Intel RealSense depth camera to detect and navigate to pallets. The project includes both the original Python implementation and a complete C++ port.

## Directory Structure

- `python_original/`: Contains the original Python source code, dependencies list, and README.
- `cpp_port/`: Contains the C++ port of the project, including all source code and build files.

---

## Python Version (`python_original/`)

The original implementation of the forklift control logic.

### Setup and Dependencies

1.  **Navigate to the Python directory:**
    ```bash
    cd python_original
    ```

2.  **Create a virtual environment:**
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install dependencies:**
    The required packages are listed in `requirements.txt`. Install them using pip:
    ```bash
    pip install -r requirements.txt
    ```
    *Note: This requires that you have the correct system libraries for `pyrealsense2` and `torch`.*

### Running the Scripts

-   **Pallet Tracking Mode:** This script autonomously navigates the forklift to align with a detected pallet.
    ```bash
    python pallet_tracking.py
    ```

-   **Pallet Forking Mode:** This script executes a pre-defined sequence to lift a pallet from a vertical stack.
    ```bash
    python pallet_forking.py
    ```

---

## C++ Version (`cpp_port/`)

A complete C++ port of the original Python project, designed for higher performance and lower-level hardware control.

### Dependencies

To build the C++ version, you will need the following libraries:

-   **CMake** (Build system)
-   **Eigen3** (Linear algebra library)
-   **Intel RealSense SDK (librealsense)** (Camera interface)
-   **OpenCV** (Computer vision library)

On macOS, these can be easily installed using [Homebrew](https://brew.sh/):
```bash
brew install cmake eigen librealsense opencv
```

### Building the Project

1.  **Navigate to the C++ directory:**
    ```bash
    cd cpp_port
    ```

2.  **Configure with CMake:**
    This will create a `build` directory and prepare the project for compilation.
    ```bash
    cmake -B build
    ```

3.  **Compile the code:**
    ```bash
    cmake --build build
    ```
    The final executable will be located at `cpp_port/build/bin/AutonomousForklift`.

### Running the Application

**IMPORTANT: Provide ONNX Models**
Before running, you must convert the original PyTorch models (`.pt`) to ONNX format (`.onnx`). The application expects the following files in the `cpp_port` directory:
- `241129_palette.onnx` (for tracker mode)
- `240808_krri.onnx` (for forker mode)

If your models are in a different location, update the paths in `cpp_port/src/main.cpp`.

**Run from the project root directory (`Autonomous_forklift_source/`):**

-   **Pallet Tracking Mode:**
    ```bash
    ./cpp_port/build/bin/AutonomousForklift tracker
    ```

-   **Pallet Forking Mode:**
    ```bash
    ./cpp_port/build/bin/AutonomousForklift forker
    ```

The application will start, display the camera feed with detections, and print mock CAN control messages to the console. Log files and video output will be saved in a new `run_<timestamp>` directory created inside `cpp_port/`.
