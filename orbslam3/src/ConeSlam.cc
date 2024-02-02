/**
* The purpose of this class...
*/
#include <iostream>
#include "ConeSlam.h"

namespace ORB_SLAM3
{

ConeSlam::ConeSlam(): hit_counter(0){}

ConeSlam::ConeSlam(double x, double y, double z, int class_id): x(x), y(y), z(z), class_id(class_id), hit_counter(0){}

// Getter methods
int ConeSlam::getClassId() const {
    return class_id;
}

double ConeSlam::getX() const {
    return x;
}

double ConeSlam::getY() const {
    return y;
}

double ConeSlam::getZ() const {
    return z;
}

int ConeSlam::getHitCounter() const {
    return hit_counter;
}

// Setter methods
void ConeSlam::setClassId(int id) {
    class_id = id;
}

void ConeSlam::incrementHitCounter(){
    hit_counter++;
}

void ConeSlam::setX(double newX) {
    x = newX;
}

void ConeSlam::setY(double newY) {
    y = newY;
}

void ConeSlam::setZ(double newZ) {
    z = newZ;
}

} //namespace ORB_SLAM
