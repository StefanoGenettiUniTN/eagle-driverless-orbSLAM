/**
* The purpose of this class...
*/
#include <iostream>
#include <set>
#include "Circuit.h"

namespace ORB_SLAM3
{

Circuit::Circuit(){
    numCones = 0;
}

void Circuit::addCone(ConeSlam _cone){
    ConeSlam* newCone = new ConeSlam(_cone.x, _cone.y, _cone.z, _cone.class_id);

    double threshold = 0.15;
    bool coneAlreadyPresent = false;
    ConeSlam* matchCone;
    for (ConeSlam* cone : cones) {
        double distance = newCone->distance(*cone);
        if(distance<threshold && _cone.class_id==cone->class_id){
            coneAlreadyPresent = true;
            matchCone = cone;
            break;
        }
    }

    if(coneAlreadyPresent==false){
        cones.push_back(newCone);
        newCone->incrementHitCounter();
        numCones++;
    }else{
        
        // refine cone coordinates
        matchCone->x = ( (matchCone->x) + (newCone->x) ) / 2.0;
        matchCone->y = ( (matchCone->y) + (newCone->y) ) / 2.0;
        matchCone->z = ( (matchCone->z) + (newCone->z) ) / 2.0;

        matchCone->incrementHitCounter();
    }

    return;
}

/*
void Circuit::removeCone(ConeSlam* _cone){
    cones.erase(_cone);
}
*/

void Circuit::removeDuplicates(){
    double threshold = 0.2;
    std::vector<int> conesToRemove;
    for(int i=0; i<cones.size(); i++){
        ConeSlam* c1 = cones[i];
        if(c1->getHitCounter()>1){
            for(int j=i+1; j<cones.size(); j++){
                ConeSlam* c2 = cones[j];
                if(c2->getHitCounter()>1){
                    double distance = c1->distance(*c2);
                    if(distance<threshold){
                        if((c1->getHitCounter()) > (c2->getHitCounter())){
                            conesToRemove.push_back(j);
                        }else{
                            conesToRemove.push_back(i);
                        }
                    }
                }
            }
        }
    }

    for(int coneToRemoveIndex : conesToRemove){
        cones[coneToRemoveIndex]->hit_counter = 0;
        numCones--;
    }
}

void Circuit::printCones(){
    for (const auto& cone : cones) {
        std::cout << "(" << cone->x << ", " << cone->y << ", " << cone->z << ")\n";
    }
}

void Circuit::circuitEnhancement(){
    std::cout<<"CIRCUIT ENHANCEMENT"<<std::endl;

    std::cout<<"remove duplicates"<<std::endl;
    removeDuplicates();

    std::cout<<"remove outliers"<<std::endl;
    for(auto it = cones.begin(); it != cones.end();){
        if ((*it)->hit_counter < 5) {
            it = cones.erase(it);
        } else {
            ++it;
        }
    }

    std::cout<<"CIRCUIT ENHANCEMENT DONE"<<std::endl;
}


int Circuit::getCircuitSize(){
    return numCones;
}


} //namespace ORB_SLAM
