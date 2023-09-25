/**
* Offline D455
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

    // Declare RealSense pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_device_from_file(sequence_file_name);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    //Start the pipeline with the set configurations
    pipe.start(cfg);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //nota:
    //  argv[1] --> vocabulary
    //  argv[2] --> camera configuration file yaml
    //  ORB_SLAM2::System::RGBD --> sensor to be used
    //  true --> const bool bUseViewer; if true the Viewer thread is itialized and launched
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    rs2::frameset data;
    cv::Mat imRGB, imD;
    double tframe;
    while (waitKey(1)) //iterate over all frames
    {   
        data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        // Read image and depthmap
        rs2::frame color_frame = data.get_color_frame();
        rs2::depth_frame depth_frame = data.get_depth_frame();
        imRGB = Mat(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        imD = Mat(Size(640, 480), CV_8UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        tframe = data.get_timestamp();

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackRGBD(imRGB,imD,tframe);

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
        /* PRINT CURRENT CAMERA POSE
        cout<<"Tcw:"<<endl;
        cout<<Tcw<<endl;
        cout<<endl;
        */

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame
        /*
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;   //nota: tframe is the timestamp of the current frame
                                            //vTimestamps[ni+1] is the timestamp of the subsequent frame
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)    //nota: if the time spent by orbslam to do the tracking is smaller than the difference
                        //between the timestamp of the next frame and the current timestamp, we wait 
            usleep((T-ttrack)*1e6);
        */
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());    //npta: the sort is computed to compute the media below
    float totaltime = 0;
    for(float time : vTimesTrack)
    {
        totaltime+=time;
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;

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