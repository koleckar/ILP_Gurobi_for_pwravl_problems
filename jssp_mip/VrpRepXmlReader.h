#pragma once

#include "tinyxml.h"

#include <vector>
#include <string>


/**
 Class facilitating of reading the .xml datasets in "VRP-REP" format.
 */
class VrpRepXmlReader {
    TiXmlDocument xmlFile;
    TiXmlHandle xmlFileHandle;

public:
    std::string datasetName;

    VrpRepXmlReader(const char *filePath);

    void getDatasetName();

    int getNumberOfNodes();

    int getFleetSize();

    std::vector<std::pair<float, float>> getNodesCoordinates(int numberOfNodes);

    std::vector<std::pair<float, float>> getTimeWindows(int numberOfNodes);

    std::vector<float> getServiceTimes(int numberOfNodes);

    std::vector<float> getNodeDemands(int numberOfNodes);

    float getVehicleCapacity();
};

