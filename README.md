# eagle-driverless-orbSLAM
Computer Vision course project 2023 - Simultaneous Localization and Mapping for Formula Student Driverless Race Car

# Branch: D455 offline
The purpose of this branch is to try to execute orbslam2 on a video acquired with our camera: Intel Realsense D455.

## folder structure
- `realsense-get-started`: contains the basic code to use the realsense library to acquire frames and display both color and depth information. For usage instructions, see the dedicated section below.
- `orbslam2`: contains the implementation of orbslam2 as proposed in the [official repository](https://github.com/raulmur/ORB_SLAM2), together with:
  -  the implementation of save map
  -  meaningful comments to understand the code
- `./Examples/d455-offline/d455_offline1`: code to execute orbslam2 on a video stream acquired with our realsense D455 camera. In order to execute the code type the following:
```
./Examples/d455-offline/d455_offline1 Vocabulary/ORBvoc.txt ./Examples/d455-offline/d455_openCV.yaml <path .bag file>
```
- `./Examples/d455-offline/d455_offline`: old version of d455_offline1 file. We keep the file for reference but from Sept the 25th we consider d455_offline1 version of the file.

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

## orbslam2
### RGBD Example
1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.
2. Execute the example, type the following. Change `TUMX.yaml` to `TUM1.yaml`, `TUM2.yaml` or `TUM3.yaml` for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER` to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.
```
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```

```
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml Examples/RGB-D/rgbd_dataset_freiburg1_xyz/ Examples/RGB-D/associations/fr1_xyz.txt
```
