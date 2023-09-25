/**
* I try to include this implementation to understand whether the
* algorithm performs better.
* Credits: https://github.com/GuoPingPan/ORB_SLAM2_With_Realsense/blob/main/Examples/RGB-D/rgbd_video.cc
*
* From Sept the 25th I would suggest to conisider this file (d455_offline1.cc)
* instead of the d455_offline.cc version.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <sstream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp>

#include <System.h>

using namespace std;
using namespace cv;

class RealsenseCam{
public:
    RealsenseCam();
    RealsenseCam(string filename);
    const vector<cv::Mat>* getFrames();

    int imageW;
    int imageH;

    vector<cv::Mat> frames;

private:
    rs2::pipeline pipe;
    rs2::align align_to_depth = rs2::align(RS2_STREAM_COLOR);
    rs2::frameset frameset;
    rs2::config cfg;

};

RealsenseCam::RealsenseCam() {
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_ANY,60);
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_ANY,60);
    pipe.start(cfg);
    cout<<"RealsenseCamera is initializing ......";
    //culling first five frame.
    frameset = pipe.wait_for_frames();
    while(frameset == NULL)
        frameset = pipe.wait_for_frames();
    frameset = align_to_depth.process(frameset);

    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();

    imageW = depth.as<rs2::video_frame>().get_width();
    imageH = depth.as<rs2::video_frame>().get_height();
    cv::Mat depthCV{cv::Size(imageW,imageH),CV_16UC1,const_cast<void*>(depth.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat colorCV{cv::Size(imageW,imageH),CV_8UC3,const_cast<void*>(color.get_data()),cv::Mat::AUTO_STEP};

    frames.push_back(depthCV);
    frames.push_back(colorCV);

}

RealsenseCam::RealsenseCam(string filename) {
    cfg.disable_all_streams();
    //cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_ANY,60);
    //cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_ANY,60);
    cfg.enable_device_from_file(filename);

    pipe.start(cfg);
    cout<<"RealsenseCamera is initializing ......";

    //culling first five frame.
    frameset = pipe.wait_for_frames();
    while(frameset == NULL)
        frameset = pipe.wait_for_frames();
    frameset = align_to_depth.process(frameset);

    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();

    imageW = depth.as<rs2::video_frame>().get_width();
    imageH = depth.as<rs2::video_frame>().get_height();
    cv::Mat depthCV{cv::Size(imageW,imageH),CV_16UC1,const_cast<void*>(depth.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat colorCV{cv::Size(imageW,imageH),CV_8UC3,const_cast<void*>(color.get_data()),cv::Mat::AUTO_STEP};

    frames.push_back(depthCV);
    frames.push_back(colorCV);

}

const vector<cv::Mat>* RealsenseCam::getFrames() {
    frameset = pipe.wait_for_frames();
    frameset = align_to_depth.process(frameset);
    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();
    cv::Mat depthCV{cv::Size(imageW,imageH),CV_16UC1,const_cast<void*>(depth.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat colorCV{cv::Size(imageW,imageH),CV_8UC3,const_cast<void*>(color.get_data()),cv::Mat::AUTO_STEP};

    frames.clear();
    frames.push_back(depthCV);
    frames.push_back(colorCV);
    return &frames;
}


int main(int argc, char **argv)
{
    /*
    nota:
        argv[1] --> vocabulary
        argv[2] --> camera configuration file yaml
        argv[3] --> path_to_sequence
    */
    if(argc != 4)
    {
        cerr << endl << "Usage: ./d455_offline path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    string sequence_file_name = string(argv[3]);
    cout<<"sequence file: "<<sequence_file_name<<endl;

    RealsenseCam realsenseCam(sequence_file_name);

    const vector<cv::Mat>* frames;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    cv::Mat imRGB, imD;

    while(1){
        frames = realsenseCam.getFrames();
        imD = (*frames)[0];
        imRGB = (*frames)[1];
        clock_t tframe = clock();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // print tracking time in ms
		//cout<<"This ttrack use " << ttrack << "ms"<<endl;

        if(cv::waitKey(33)=='q')
           break;
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    /*
    nota:
    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    */

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt"); //nota: in this case the file is in line with the TUM dataset format. It is not mandatory to save the trajectory in this way.
                                                    //  first line: 1305031102.175304 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
                                                    //  subsequent line example: 
                                                    //      1305031102.211214 -0.001603064 0.008035151 0.010883896 -0.001937729 -0.002992724 -0.001707897 0.999992192
                                                    //      timestamp         tx           ty          tz           qx           qy           qz          qw           
                                                    //
                                                    //timestamp (float) gives the number of seconds since the Unix epoch.
                                                    //tx ty tz (3 floats) give the position of the optical center of the color camera with respect to the world origin as defined by the motion capture system.
                                                    //qx qy qz qw (4 floats) give the orientation of the optical center of the color camera in form of a unit quaternion with respect to the world origin as defined by the motion capture system


    /*
    nota:
    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    */
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    /*nota:
        With the following function we save the map in a file as proposed
        by ProgrammerAll webpage: https://www.programmerall.com/article/18761312556/
    */   
    SLAM.SaveMap("MyMap.bin");

    return 0;
}