/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>

#include<opencv2/core/core.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    cout<<"ciao ciao"<<endl;

    /*
    nota:
        argv[1] --> vocabulary
        argv[2] --> camera configuration file yaml
        argv[3] --> path_to_sequence
        argv[4] --> path_to_association
    */
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);    //nota: associate RGB images and depth images 
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //nota:
    //  argv[1] --> vocabulary
    //  argv[2] --> camera configuration file yaml
    //  ORB_SLAM2::System::RGBD --> sensor to be used
    //  true --> const bool bUseViewer; if true the Viewer thread is itialized and launched
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++) //nota: iterate over all frames
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

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

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;   //nota: tframe is the timestamp of the current frame
                                            //vTimestamps[ni+1] is the timestamp of the subsequent frame
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)    //nota: if the time spent by orbslam to do the tracking is smaller than the difference
                        //between the timestamp of the next frame and the current timestamp, we wait 
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());    //npta: the sort is computed to compute the media below
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

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

    return 0;
}

/*

nota:
A line of the association file is for example like this:
1305031102.175304 rgb/1305031102.175304.png 1305031102.160407 depth/1305031102.160407.png
timestamp_rgb   img_rgb_fileName    timestamp_depth     img_depth_fileName
This function populate
    -> vTimestamps with the rgb timestamp values
    -> vstrImageFilenamesRGB with the names of the rgb image files
    -> vstrImageFilenamesD with the names of the depth image files
*/
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
