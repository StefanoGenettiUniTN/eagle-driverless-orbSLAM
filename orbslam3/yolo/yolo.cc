#include <iostream>
#include <stdio.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "inference.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    string videoFile;

    if(argc!=2){
        cout<<"please provide a rosbag video stream to be processed with YOLOv8"<<endl;
        return -1;
    }

    // the program requires a single argument which is the path of the video stream file (rosbag format is required)
    videoFile = argv[1];
    cout<<"video file: "<<videoFile<<endl;

    // check opencv version
    cout<<"opencv version: "<<CV_VERSION<<endl;

    // Declare frameset and frames which will hold the data from the camera
    rs2::frameset frames;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    rs2::config cfg;
    cfg.enable_device_from_file(videoFile);

    // Configure and start the pipeline
    p.start(cfg);

    cout<<"the pipeline has started!"<<endl;

    //Block program until frames arrive
    frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    // Try to get a frame of a color image
    rs2::frame color_frame = frames.get_color_frame();
    
    // Get the depth frame's dimensions
    float depth_width = depth_frame.get_width();
    float depth_height = depth_frame.get_height();

    // TODO: Get the color frame's dimensions
    float color_width = 0.0;
    float color_height = 0.0;

    cout<<"Color frame: "<<endl;
    cout<<"width: TODO"<<endl;
    cout<<"height: TODO"<<endl<<endl;

    cout<<"Depth frame: "<<endl;
    cout<<"width: "<<depth_width<<endl;
    cout<<"height: "<<depth_width<<endl<<endl;

    // Creating OpenCV Matrix from a color image
    Mat color(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    // Creating OpenCV matrix from IR image
    Mat ir(Size(420, 240), CV_8UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

    // Apply Histogram Equalization
    equalizeHist( ir, ir );
    applyColorMap(ir, ir, COLORMAP_JET);

    imshow("Display Image", color);

    waitKey(0);

    imshow("Display depth", ir);

    waitKey(0);
    
    // LOAD YOLO MODEL
    cout<<"Loading YOLO model ..."<<endl;
    string onnx_model_path = "models/best-seg-480-640.onnx";
    Size modelInputShape(640, 480);
    string classesTxtFile = "classes.txt";
    bool runWithCuda = true;

    YOLO::Inference inf(onnx_model_path, modelInputShape, classesTxtFile, runWithCuda);

    // ... END LOAD YOLO MODEL

    cout<<"video flow: "<<endl;

    // Video flow
    while (waitKey(1))
    {
        rs2::frameset data = p.wait_for_frames(); // Wait for next set of frames from the camera
        depth_frame = data.get_depth_frame();
        color_frame = data.get_color_frame();

        // Creating OpenCV Matrix from a color image
        Mat color_frame_mat(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(color_frame_mat, color_frame_mat, cv::COLOR_BGR2RGB);

        // Creating OpenCV matrix from IR image
        // and apply Histogram Equalization
        Mat ir(Size(848, 848), CV_8UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        equalizeHist( ir, ir );
        applyColorMap(ir, ir, COLORMAP_JET);

        // Update the window with new data
        imshow("Display Image", color_frame_mat);
        //imshow("Display depth", ir);

        // YOLO INFERENCE
        cv::resize(color_frame_mat, color_frame_mat, cv::Size(640, 480));
        imshow("Display Image resized", color_frame_mat);
        std::vector<YOLO::Detection> output = inf.runInference(color_frame_mat);
        
        int detections = output.size();
        cout<<"Number of detections: "<<detections<<endl;

        for (int i=0; i<detections; ++i){
            YOLO::Detection detection = output[i];

            cv::Rect box = detection.box;
            cv::Scalar color = detection.color;
            cv::Mat mask = detection.boxMask;

            // Detection box
            cv::rectangle(color_frame_mat, box, color, 2);

            // Detection mask
            color_frame_mat(box).setTo(color, mask);

            // Detection box text
            std::string classString = detection.className + ' ' + to_string(detection.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

            cv::rectangle(color_frame_mat, textBox, color, cv::FILLED);
            cv::putText(color_frame_mat, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
        }

        // This is only for preview purposes
        float scale = 1;
        cv::resize(color_frame_mat, color_frame_mat, cv::Size(color_frame_mat.cols*scale, color_frame_mat.rows*scale));
        cv::imshow("Inference", color_frame_mat);

        // ...END YOLO INFERENCE
    }

    return 0;
}