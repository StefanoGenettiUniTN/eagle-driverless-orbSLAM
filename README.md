# eagle-driverless-orbSLAM
Computer Vision course project 2023 - Simultaneous Localization and Mapping for Formula Student Driverless Race Car

# Branch: D455 offline
The purpose of this branch is to try to execute orbslam2 on a video acquired with our camera: Intel Realsense D455.

## folder structure
- `realsense-get-started`: contains the basic code to use the realsense library to acquire frames and display both color and depth information. For usage instructions, see the dedicated section below.

## realsense-get-started
The folder contains two files: `bag.cc`, `pipeline.cc`. The former is to process a pre-recorder video stream, while the latter is the skeleton code to process an online stream of data.
The code in `bag.cc` has been tested on a video acquired with `realsense-viewer` which outputs a `.bag` file. If you want to try `bag.cc` execution on your own acquisition, record a video with `realsense-viewer` and save it in a `.bag` file. Then replace the following line of code with your own file path:
```
rs2::config cfg;
cfg.enable_device_from_file("../20230525_110936.bag");
```
To compile the code write the following:
```
g++ -std=c++11 bag.cc -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc
```
In alternative, you can prepare a CMake file.

Execution:
```
./a.out
```
