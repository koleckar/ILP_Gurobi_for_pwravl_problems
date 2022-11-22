#pragma once

#include "LoaderJSPLIB.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>


using std::vector;
using std::string;


JSPLIBInstance LoaderJSPLIB::loadInstance(const string& path) {
    std::ifstream input(path);
    std::string line;

    getline(input, line);
    getline(input, line);
    string datasetName = line;
    getline(input, line);
    getline(input, line);
    getline(input, line);

    std::istringstream is(line);
    vector<int> parsedLine((std::istream_iterator<int>(is)), (std::istream_iterator<int>()));
    int numberOfJobs = parsedLine[0];
    int numberOfMachines = parsedLine[1];

    vector<vector<int>> J(numberOfJobs, vector<int>(numberOfMachines)); // jobs precedence matrix
    vector<vector<int>> P(numberOfJobs, vector<int>(numberOfMachines)); // job duration on machine

    for (int jobID = 0; jobID < numberOfJobs; jobID++)
    {
        getline(input, line);

        std::istringstream is(line);
        vector<int> parsedLine((std::istream_iterator<int>(is)), (std::istream_iterator<int>()));

        for (int lineIdx = 0; lineIdx < numberOfJobs * 2; lineIdx += 2)
        {
            int machineID = parsedLine[lineIdx];
            int jobDuration = parsedLine[lineIdx + 1];

            J[jobID][lineIdx / 2] = machineID;
            P[jobID][machineID] = jobDuration;
        }
    }
    return JSPLIBInstance{datasetName, numberOfJobs, numberOfMachines, J, P };
}

