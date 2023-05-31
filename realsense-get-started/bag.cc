/*
    Realsense hello world
    https://github.com/IntelRealSense/librealsense/tree/master/examples/hello-realsense
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <sstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    cout<<"Realsense hello world example"<<endl;

    //Check opencv version
    printf("%s\r\n", CV_VERSION);

    // Declare frameset and frames which will hold the data from the camera
    rs2::frameset frames;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    rs2::config cfg;
    cfg.enable_device_from_file("../20230525_110936.bag");

    // Configure and start the pipeline
    p.start(cfg);

    cout<<"test"<<endl;

    //Block program until frames arrive
    frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    // Try to get a frame of a color image
    rs2::frame color_frame = frames.get_color_frame();
    
    // Get the depth frame's dimensions
    float depth_width = depth_frame.get_width();
    float depth_height = depth_frame.get_height();

    cout<<"Depth frame: "<<endl;
    cout<<"width: "<<depth_width<<endl;
    cout<<"height: "<<depth_width<<endl<<endl;

    // Creating OpenCV Matrix from a color image
    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    // Creating OpenCV matrix from IR image
    Mat ir(Size(640, 480), CV_8UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    // Apply Histogram Equalization
    equalizeHist( ir, ir );
    applyColorMap(ir, ir, COLORMAP_JET);

    imshow("Display Image", color);

    waitKey(0);

    imshow("Display depth", ir);

    waitKey(0);

    cout<<"video flow: "<<endl;

    // Video flow
    while (waitKey(1))
    {
        rs2::frameset data = p.wait_for_frames(); // Wait for next set of frames from the camera
        depth_frame = data.get_depth_frame();
        color_frame = data.get_color_frame();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(color, color, cv::COLOR_BGR2RGB);

        // Creating OpenCV matrix from IR image
        // and apply Histogram Equalization
        Mat ir(Size(640, 480), CV_8UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        equalizeHist( ir, ir );
        applyColorMap(ir, ir, COLORMAP_JET);

        // Update the window with new data
        imshow("Display Image", color);
        //imshow("Display depth", ir);
    }

    return 0;
}
