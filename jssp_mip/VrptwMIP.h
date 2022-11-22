#pragma once

#include "ModelMIP.h"

#include <vector>


using std::vector;
using std::pair;

class VrptwMIP : public ModelMIP  {
private:
    void twoIndexVehicleFlowFormulation(int numberOfNodes, int vehicleCapacity, int fleetSize, 
                                        vector<vector<int>> distanceMatrix,
                                        vector<int> demands, 
                                        vector<pair<int, int>> timeWindows, 
                                        float timeLimit);

    void threeIndexVehicleFlowFormulation(int numberOfNodes, int vehicleCapacity, int fleetSize,
                                          vector<vector<int>> distanceMatrix,
                                          vector<int> demands,
                                          vector<pair<int, int>> timeWindows,
                                          float timeLimit);

public:
    void solveInstance(const char* instancePath, float timeLimit);

};

