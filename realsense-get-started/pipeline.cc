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

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

using namespace std;


int main(int argc, char **argv)
{
    cout<<"Realsense hello world example"<<endl;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    cout<<"test"<<endl;

    //Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    // Get the depth frame's dimensions
    float width = depth.get_width();
    float height = depth.get_height();

    cout<<"width: "<<width<<endl;
    cout<<"height: "<<height<<endl;

    return 0;
}
