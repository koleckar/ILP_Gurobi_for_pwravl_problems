#pragma once

#include "VrpRepXmlReader.h"

#include <vector>
#include <cmath>


using std::vector;
using std::pair;


struct VrptwInstance {
    /// <summary>
    /// Instance of Vehicle Routing Problem with Time Windows from VRP-REP.
    /// </summary>
    int numberOfNodes;
    int vehicleCapacity;
    int fleetSize;
    vector<vector<int>> distanceMatrix;
    vector<int> demands;
    vector<pair<int, int>> timeWindows;
};




inline float euclideanDistance2D(pair<float, float>& x, pair<float, float>& y) {
    return std::sqrt(std::pow(x.first - y.first, 2) + std::pow((x.second - y.second), 2));
}


vector<vector<int>> createDistanceMatrixFromCoordinates(vector<pair<float, float>>&& coordinates) {

    vector<vector<int>> distanceMatrix
    (coordinates.size(), vector<int>(coordinates.size()));

    int i = 0, j = 0;
    for (auto& coord1 : coordinates) {
        for (auto& coord2 : coordinates) {
            if (i == j) {
                distanceMatrix[i][j] = 0;
            }
            else {
                int distance = round(euclideanDistance2D(coord1, coord2));

                distanceMatrix[i][j] = distance;
                distanceMatrix[j][i] = distance;
            }
            j++;
        }
        i++;
        j = 0;
    }

    return distanceMatrix;
}
// todo: code would be cleaner if first datastructures made N+1.
VrptwInstance getInstance(const char* instancePath) {
    /// Load data. All values are integers, if not they are rounded.

    VrpRepXmlReader vrpReader(instancePath);

    int N = vrpReader.getNumberOfNodes(); // Representation is: start at depot 0, return to depot N+1.

    int Q = vrpReader.getVehicleCapacity();

    int K = vrpReader.getFleetSize();

    vector<vector<int>> distanceMatrix  // Distances are rounded to the closest int.
        = createDistanceMatrixFromCoordinates(vrpReader.getNodesCoordinates(N));

    vector<int> demands;
    demands.reserve(N);
    for (auto nodeDemand : vrpReader.getNodeDemands(N)) {
        demands.emplace_back((int)nodeDemand);
    }

    vector<pair<int, int>> timeWindows;
    timeWindows.reserve(N);
    for (auto timeWindow : vrpReader.getTimeWindows(N)) {
        timeWindows.emplace_back(timeWindow.first, timeWindow.second);
    }
    timeWindows[0] = pair<int, int>{ 0, std::numeric_limits<int>::max() };

    return { N, Q, K, distanceMatrix, demands, timeWindows };
}

