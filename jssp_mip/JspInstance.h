#pragma once

#include <vector>
#include <string>

struct JSPLIBInstance {
    /// <summary>
    /// Job shop instance from JSPLIB.
    /// </summary>
    std::string instanceName;
    int numberOfJobs;
    int numberOfMachines;
    std::vector< std::vector<int>> precedencesMatrix;
    std::vector< std::vector<int>> durationsMatrix;
};


