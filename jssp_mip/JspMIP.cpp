#pragma once

#include "LoaderJSPLIB.h"
#include "JspMIP.h"

#include "gurobi_c++.h"

#include <numeric>
#include <algorithm>
#include <string>

using std::string;
using std::cout;
using std::endl;


void JspMIP::solveInstance(const char* instancePath, float timeLimit) {
    // "Disjunctive model" as given in 

    // ------ Load JSP instance and create bounds. ---------------
    const JSPLIBInstance instance = LoaderJSPLIB::loadInstance(instancePath);

    int minCmax = 0;
    int bigM = 0;

    for (int i = 0; i < instance.numberOfJobs; i++) {
        int sum = std::accumulate(std::begin(instance.durationsMatrix[i]), std::end(instance.durationsMatrix[i]), 0);

        minCmax = std::max(sum, minCmax);

        bigM += sum;
    }

    cout << '\n';

    // ------ Gurobi model. ---------------
    GRBEnv* env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.set(GRB_StringAttr_ModelName, "JSPLib MIP solver");
    model.set(GRB_DoubleParam_TimeLimit, timeLimit);


    // ------ Variables. ---------------
    GRBVar Cmax = model.addVar(minCmax, GRB_INFINITY, 1, GRB_CONTINUOUS, "Cmax"); // lb, ub, obj, type, name

    GRBVar** x = new GRBVar * [instance.numberOfJobs]; // x[jobID][machineID] = start of job on machine
    for (int i = 0; i < instance.numberOfJobs; i++) {
        x[i] = new GRBVar[instance.numberOfMachines];
    }
    for (int i = 0; i < instance.numberOfJobs; i++) {
        for (int j = 0; j < instance.numberOfMachines; j++) {
            string name = "x" + std::to_string(i) + std::to_string(j);
            x[i][j] = model.addVar(0, GRB_INFINITY, 0, GRB_INTEGER, name);
        }
    }

    GRBVar*** z = new GRBVar * *[instance.numberOfMachines];
    for (int i = 0; i < instance.numberOfMachines; i++) {
        z[i] = new GRBVar * [instance.numberOfJobs];
        for (int j = 0; j < instance.numberOfJobs; j++) {
            z[i][j] = new GRBVar[instance.numberOfJobs];
        }
    }


    for (int i = 0; i < instance.numberOfMachines; i++) {
        for (int j = 0; j < instance.numberOfJobs; j++) {
            for (int k = 0; k < instance.numberOfJobs; k++) {
                string name = "z" + std::to_string(i) + std::to_string(j) + std::to_string(k);
                z[i][j][k] = model.addVar(0, 1, 0, GRB_BINARY, name);
            }
        }
    }


    // ------ Constraints. ---------------
    for (int jobID = 0; jobID < instance.numberOfJobs; jobID++) {
        for (int i = 1; i < instance.numberOfMachines; i++) {
            int machineIDprev = instance.precedencesMatrix[jobID][i - 1];
            int machineID = instance.precedencesMatrix[jobID][i];
            model.addConstr(x[jobID][machineID] >= x[jobID][machineIDprev] + instance.durationsMatrix[jobID][machineIDprev]);
        }
    }

    for (int jobID = 0; jobID < instance.numberOfJobs; jobID++) {
        int lastMachineID = instance.precedencesMatrix[jobID][instance.numberOfMachines - 1];
        model.addConstr(Cmax >= x[jobID][lastMachineID] + instance.durationsMatrix[jobID][lastMachineID]);
    }

    for (int machineID = 0; machineID < instance.numberOfMachines; machineID++) {
        for (int j = 0; j < instance.numberOfJobs; j++) {
            for (int k = 0; k < instance.numberOfJobs; k++) {
                if (j == k) {
                    continue;
                }
                model.addConstr(x[j][machineID] >= x[k][machineID] + instance.durationsMatrix[k][machineID] - bigM * z[machineID][j][k]);
                model.addConstr(x[k][machineID] >= x[j][machineID] + instance.durationsMatrix[j][machineID] - bigM * (1 - z[machineID][j][k]));
            }
        }
    }


    //------- Solve the model. -----------
    model.optimize();

    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
        cout << "\n==================================" << endl;
        cout << "Cmax: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        for (int i = 0; i < instance.numberOfMachines; i++) {
            for (int j = 0; j < instance.numberOfMachines; j++) {
                cout << x[i][j].get(GRB_DoubleAttr_X) << " ";
            }
            cout << endl;
        }
        cout << "==================================\n";
    }
    else {
        model.computeIIS();
        model.write("jsp_model_IIS.ilp");
    }

}



