/**
* The purpose of this class is...
*/


#ifndef CIRCUIT_H
#define CIRCUIT_H

#include <set>
#include <vector>
#include <cmath>
#include "ConeSlam.h"
#include "Circuit.h"

namespace ORB_SLAM3
{
class Circuit{
public:
    Circuit();

    // set of cones which populate the circuit
    std::vector<ConeSlam*> cones;

    // number of cones which populate the circuit
    int numCones;

    // add a cone to the circuit
    // if the cone has already been inserted, refine its coordinates
    void addCone(ConeSlam _cone);

    // remove a cone from the circuit
    ////void removeCone(ConeSlam* _cone);

    // remove cone duplicates from the circuit
    void removeDuplicates();

    // print the cones of the circuit
    void printCones();

    // circuit enhancement
    // this function is called in order to enhance the collection of cone
    // clusters (postprocessing) and fill the final output cone csv file
    void circuitEnhancement();

    // get cone number
    int getCircuitSize();
};
} //namespace ORB_SLAM
#endif // CIRCUIT_H
