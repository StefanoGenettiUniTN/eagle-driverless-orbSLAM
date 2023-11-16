/**
* This file is NOT part of ORB-SLAM2.
*
* In this file we create a Camera object which contains information about
* the pose of the Camera (x,y,z)+(quaternion).
*/

#ifndef CAMERA_POSE_H
#define CAMERA_POSE_H
namespace ORB_SLAM2
{
class CameraPose
{
public:
    CameraPose();

    // This contains the position of the camera in the free space
    double x;
    double y;
    double z;

    // This represents the orientation of the camera in free space in quaternion form.
    double q_x;
    double q_y;
    double q_z;
    double q_w;
};
} // namespace ORB_SLAM2
#endif // CAMERA_POSE_H
