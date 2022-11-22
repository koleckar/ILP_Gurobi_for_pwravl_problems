#pragma once

# include "VrptwMIP.h"
# include "VrptwInstance.h"
# include "VrpRepXmlReader.h"

# include <gurobi_c++.h>

# include <vector>
# include <cmath>
# include <limits>

using std::vector;
using std::pair;
using std::string;
using std::cout;
using std::endl;


void  VrptwMIP::solveInstance(const char* instancePath, float timeLimit) {
    VrptwInstance instance = getInstance(instancePath);
    
    twoIndexVehicleFlowFormulation(instance.numberOfNodes, instance.vehicleCapacity, instance.fleetSize, 
                                   instance.distanceMatrix, instance.demands, instance.timeWindows, timeLimit);

}


void VrptwMIP::twoIndexVehicleFlowFormulation(int numberOfNodes, int vehicleCapacity, int fleetSize,
                                              vector<vector<int>> distanceMatrix, 
                                              vector<int> demands,
                                              vector<pair<int, int>> timeWindows, 
                                              float timeLimit){
    /// MIP two-index flow formulation for C-VRP-TW, as in https://arxiv.org/pdf/1606.01935.pdf, pg.4, equations (2.1)-(2.9).

    // ------ Gurobi model. ---------------
    GRBEnv *env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.set(GRB_StringAttr_ModelName, "VRP-TW ILP model");
    model.set(GRB_DoubleParam_TimeLimit, timeLimit);



    // ------ Variables. ---------------
    GRBVar** x = new GRBVar* [numberOfNodes + 1]; // binary, x_ij == 1 iff route goes from i->j
    for (int i = 0; i < numberOfNodes + 1; ++i) {
        x[i] = new GRBVar[numberOfNodes + 1];
        for (int j = 0; j < numberOfNodes + 1; ++j) {
            string name = "x_" + std::to_string(i) + std::to_string(j);
            int objCoeffs = distanceMatrix[i % numberOfNodes][j % numberOfNodes];
            x[i][j] = model.addVar(0, 1, objCoeffs, GRB_BINARY, name);
        }
    }

    GRBVar* y = new GRBVar[numberOfNodes ];
    for (int i = 0; i < numberOfNodes + 1; ++i) {
        std::string name = "y_i" + std::to_string(i);
        y[i] = model.addVar(demands[i % numberOfNodes], vehicleCapacity, 0, GRB_CONTINUOUS, name);
    }

    GRBVar* w = new GRBVar[numberOfNodes + 1];
    for (int i = 0; i < numberOfNodes + 1; ++i) {
        std::string name = "w" + std::to_string(i);
        w[i] = model.addVar(timeWindows[i % numberOfNodes].first, timeWindows[i % numberOfNodes].second, 0, GRB_CONTINUOUS, name);
    }

    // ------ Constraints. ---------------

    for (int i = 1; i < numberOfNodes; ++i) {
        GRBLinExpr expr = 0;
        for (int j = 1; j < numberOfNodes + 1; ++j) {
            if (i == j) {continue;}
            expr += x[i][j];
        }
        std::string name = "leaving node " + std::to_string(i) + " once.";
        model.addConstr(expr == 1, name);
    }

    GRBLinExpr expr1;
    GRBLinExpr expr2;
    for (int h = 1; h < numberOfNodes; ++h) {
        expr1 = 0;
        expr2 = 0;
        for (int i = 0; i < numberOfNodes; ++i) {
            if (i == h) {continue;}
            expr1 += x[i][h];
        }
        for (int j = 1; j < numberOfNodes + 1; ++j) {
            if (j == h) {continue;}
            expr2 += x[h][j];
        }
        string name = "number of vehicles arriving and leaving node " + std::to_string(h) + " equals";
        model.addConstr(expr1 == expr2, name);
    }

    GRBLinExpr expr = 0;
    for (int j = 1; j < numberOfNodes; ++j) {
        expr += x[0][j];
    }
    model.addConstr(expr <= fleetSize, "number of vehicles leaving depot.");
    
    for (int i = 0; i < numberOfNodes + 1; ++i) {
        for (int j = 0; j < numberOfNodes + 1; ++j) {
            model.addConstr(y[j] >= y[i] + demands[j % numberOfNodes] * x[i][j] - vehicleCapacity * (1 - x[i][j]));
        }
    }

    // todo: bigM large enough scalar?
    //vector<vector<int>> bigM;
    //bigM.reserve((numberOfNodes + 1) * (numberOfNodes + 1));
    //for (int i = 0; i < numberOfNodes + 1; ++i) {
    //    for (int j = 0; j < numberOfNodes + 1; ++j) {
    //        bigM.emplace_back(std::max(0, timeWindows[i].second - timeWindows[j].first));
    //    }
    //}

    int bigM = 100000;
    for (int i = 0; i < numberOfNodes; ++i) {
        for (int j = 1; j < numberOfNodes + 1; ++j) {
            model.addConstr(w[j] >= w[i] + demands[j % numberOfNodes] * x[i][j] - bigM * (1 - x[i][j]));
        }
    }


    model.optimize();

    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
        cout << "\n==================================" << endl;
        cout << "tour cost: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        //for (int i = 0; i < numberOfNodes+1; i++) {
        //    for (int j = 0; j < numberOfNodes + 1; j++) {
        //        cout << x[i][j].get(GRB_DoubleAttr_X) << " ";
        //    }
        //    cout << endl;
        //}
        cout << "==================================\n";
    }
    else {
        model.computeIIS();
        model.write("vrptw_model_IIS.ilp");
    }

    // Print the objective
    // and the values of the decision variables in the solution.
//    std::cout << "Optimal objective: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
//    std::cout << "x: " << x.get(GRB_DoubleAttr_X) << " ";
//    std::cout << "y: " << y.get(GRB_DoubleAttr_X) << std::endl;
//    std::cout << "w: " << y.get(GRB_DoubleAttr_X) << std::endl;
}


void VrptwMIP::threeIndexVehicleFlowFormulation(int numberOfNodes, int vehicleCapacity, int fleetSize,
                                                vector<vector<int>> distanceMatrix,
                                                vector<int> demands,
                                                vector<pair<int, int>> timeWindows,
                                                float timeLimit) {
    /// as given in https://reader.elsevier.com/reader/sd/pii/S1018364710000297?token=9FADA6554ECCE12A5E12D35BD4A5B2ADD681E2213839F403847CB643836E778BF8671D023E24E2EF5CA3BB97BA423623&originRegion=eu-west-1&originCreation=20220114122755

}


// ILP formulation cubic in variables, not used.
// arc set:
// (0, i);
// (i, j): 1) a_i + t_ij <= b_j , t_ij= time from i->j + serviceTime_i , (i, n+1)
//         2) d_i + d_j <= q
// (i, n+1);
// all values are integers.
// triangle inequality on cost holds.
// start_v_1 = a_v_1
// start_v_i = max {s_v_(i-1) + t[v_(i-1),v_i], a_v_i}




// // ------ Variables. ---------------
//vector<vector<GRBVar>> x;
//x.reserve((numberOfNodes + 1)* (numberOfNodes + 1)); // binary, x_ij == 1 iff route goes from i->j
//for (int i = 0; i < numberOfNodes + 1; ++i) {
//    for (int j = 0; j < numberOfNodes + 1; ++j) {
//        string name = "x_" + std::to_string(i) + std::to_string(j);
//        int objCoeffs = distanceMatrix[i % numberOfNodes][j % numberOfNodes];
//        x.emplace_back(model.addVar(0, 1, objCoeffs, GRB_BINARY, name));
//    }
//}
//
//vector<GRBVar> y;
//y.reserve(numberOfNodes);
//for (int i = 0; i < numberOfNodes + 1; ++i) {
//    std::string name = "y_i" + std::to_string(i);
//    y.emplace_back(model.addVar(demands[i % numberOfNodes], vehicleCapacity, 0, GRB_CONTINUOUS, name.c_str()));
//}
//
//vector<GRBVar> w;
//w.reserve(numberOfNodes);
//for (int i = 0; i < numberOfNodes + 1; ++i) {
//    std::string name = "w" + std::to_string(i);
//    y.emplace_back(model.addVar(
//        timeWindows[i % numberOfNodes].first, timeWindows[i % numberOfNodes].second, 0, GRB_CONTINUOUS, name.c_str()));
//}