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

#include "CameraPose.h"

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

    // LOG FILES
    // ** camera.csv :: coordinate of the camera
    string log_camera_file_path = "report/camera.csv"; 
    // ...

    const vector<cv::Mat>* frames;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    cv::Mat imRGB, imD;

    while(1){
        frames = realsenseCam.getFrames();
        imD = (*frames)[0];
        imRGB = (*frames)[1];
        clock_t tframe = clock();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cv::Mat Tcw = SLAM.TrackRGBD(imRGB,imD,tframe);
        //SLAM.TrackRGBD(imRGB,imD,tframe);
        
        /*
        nota:
        The output cv::Mat Tcw has been added by me.
        Indeed before the return value of trackRGBD was simply
        ignored.

        At the beggining the Txw matrix is the following:
        [   1, 0, 0, 0
            0, 1, 0, 0
            0, 0, 1, 0
            0, 0, 0, 1
        ]

        Then it progressively denotes a transformation matrix from the initial pose to the current
        camera pose

        [   0.99997622,     -0.0034013058,  0.0059955055,   0.0015558163;
            0.0034244843,   0.99998671,     -0.0038600003,  -0.0079942448;
            -0.0059822965,  0.0038804403,   0.99997455,     -0.010940595;
                0,              0,              0,                 1
        ]
        */
        // PRINT CURRENT CAMERA POSE
        cout<<"Tcw:"<<endl;
        cout<<Tcw<<endl;
        cout<<endl;

        // compute camera pose
        ORB_SLAM2::CameraPose cameraPose = ORB_SLAM2::CameraPose();
        
        if(!Tcw.empty()){    // camera pose transformation matrix available
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();  // rotation information
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);        // translation information
            
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
            
            cout<<"Rwc"<<endl;
            cout<<Rwc<<endl<<endl;
            
            cout<<"twc"<<endl;
            cout<<"x: "<<twc.at<float>(0,0)<<endl;
            cout<<"y: "<<twc.at<float>(1,0)<<endl;
            cout<<"z: "<<twc.at<float>(2,0)<<endl;
            cout<<endl<<endl;

            cout<<"q"<<endl;
            for (float qi : q)
                cout << qi << ' ';
            cout<<endl;
            
            // build transformation matrix
            // TODO: non mi ricordo cosa stavamo facendo qui. Vedere se vale la
            // pena continuare. 
        }else{      // camera pose transformation matrix not available
            cout<<"no camera pose transformation matrix available"<<endl;
        }
        
        //...end compute camera pose

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // print tracking time in ms
		//cout<<"This ttrack use " << ttrack << "ms"<<endl;

        if(cv::waitKey(1)=='q')    // originally waitKey(33)
           break;
    }
    // Stop all threads
    SLAM.Shutdown();

    cout<<"Exited from main loop. Saving files."<<endl;

    // Save camera trajectory
    /*
    nota:
    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    */

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt", log_camera_file_path); //nota: in this case the file is in line with the TUM dataset format. It is not mandatory to save the trajectory in this way.
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