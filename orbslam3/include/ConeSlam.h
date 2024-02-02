/**
* The purpose of this class is...
*/

#ifndef CONE_SLAM_H
#define CONE_SLAM_H

#include <set>
#include <cmath>

namespace ORB_SLAM3
{

class ConeSlam{
public:
    // cone class
    int class_id;

    // how many times the cone has been seen by the camera
    int hit_counter;

    // cone coordinates
    double x;
    double y;
    double z;

    ConeSlam();
    ConeSlam(double x, double y, double z, int class_id);

    int getClassId() const;
    int getHitCounter() const;
    double getX() const;
    double getY() const;
    double getZ() const;

    void setClassId(int id);
    void incrementHitCounter();
    void setX(double newX);
    void setY(double newY);
    void setZ(double newZ);

    // Function to calculate the distance between two points
    double distance(const ConeSlam& other) const {
        return std::sqrt(   (x - other.x) * (x - other.x) +
                            (y - other.y) * (y - other.y) +
                            (z - other.z) * (z - other.z));
        /*
        return std::sqrt(   (x - other.x) * (x - other.x) +
                            (y - other.y) * (y - other.y));
        */
    }

    // custom comparator to compare points based on distance and yolo predicted class
    /*
    struct CompareDistance {
        bool operator()(const ConeSlam* p1, const ConeSlam* p2) const {
            // adjust this threshold as needed
            const double threshold = 0.7;
            //return p1.class_id==p2.class_id && p1.distance(p2) < threshold;
            return p1->distance(*p2) > threshold;
        }
    };
    */
};

} //namespace ORB_SLAM

#endif // CONE_SLAM_H
