#pragma once

#include "CspMIP.h"
#include "CspReader.h"
#include "CspInstance.h"

#include "gurobi_c++.h"

#include <numeric>
#include <string>
#include <vector>

using std::string;
using std::cout;
using std::endl;

CspInstance getFakeInstance() {
    int alphabetSize = 4;
    char *alphabet = new char[4]; 
    alphabet[0] = 'A';
    alphabet[1] = 'T';
    alphabet[2] = 'G';
    alphabet[3] = 'C';

    int stringLength = 10;
    int numberOfStrings = 5;

    string *strings = new string[numberOfStrings]; 
    strings[0] = "ATATGGCGCA";
    strings[1] = "GTATGGCGCC";
    strings[2] = "ATATGGCGCG";
    strings[3] = "CTATGGCGCG";
    strings[4] = "GTATGGCGCG";

    return {alphabetSize, alphabet, stringLength, numberOfStrings, strings};
}

int getSymbolIdxInAlphabet(char symbol, CspInstance& instance) {
    int idx = 0;
    for (int j = 0; j < instance.alphabetSize; j++) {
        if (instance.alphabet[j] == symbol) {
            break;
        }
        idx += 1;
    }
    return idx;
}

void CspMIP::solveInstance(const char* instancePath, float timeLimit) {
    //CspInstance instance = CspReader::loadInstance(instancePath);
    CspInstance instance = getFakeInstance();


    // ------ Gurobi model. ---------------
    GRBEnv* env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_StringAttr_ModelName, "Closest Substring Problem MIP solver.");
    model.set(GRB_DoubleParam_TimeLimit, timeLimit);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);


    // ------ Variables. ---------------
    GRBVar** x = new GRBVar * [instance.stringLength];
    for (int i = 0; i < instance.stringLength; i++) {
        x[i] = new GRBVar[instance.alphabetSize];
        for (int j = 0; j < instance.alphabetSize; j++) {
            string name = "string_" + std::to_string(i) + "_" + instance.alphabet[j];
            x[i][j] = model.addVar(0, 1, 0, GRB_BINARY, name);
        }
    }

    GRBVar d = model.addVar(0, GRB_INFINITY, 1, GRB_CONTINUOUS, "minHammingDist");


    // ------ Constraints. ---------------
    GRBLinExpr expr;
    for (int i = 0; i < instance.stringLength; i++) {
        expr = 0;
        for (int j = 0; j < instance.alphabetSize; j++) {
            expr += x[i][j];
        }
        model.addConstr(expr == 1, "only one symbol at each index.");
    }

    for (int s = 0; s < instance.numberOfStrings; s++) {
        expr = 0;
        for (int i = 0; i < instance.stringLength; i++) {
            char ch = instance.strings[s].at(i);
            int symbolIdx = getSymbolIdxInAlphabet(ch, instance);
            expr += x[i][symbolIdx];
        }
        string name = "string " + std::to_string(s) + " has correct hamming distance.";
        model.addConstr(instance.stringLength - expr <= d, name);
    }


    model.optimize();

    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
        cout << "\n==================================" << endl;
        cout << "minimal hamming distance: " << model.get(GRB_DoubleAttr_ObjVal) << endl;


        std::vector<char> t;
        t.reserve(instance.stringLength);
        for (int i = 0; i < instance.stringLength; i++) {
            for (int j = 0; j < instance.alphabetSize; j++) {
                if (x[i][j].get(GRB_DoubleAttr_X) == 1) {
                    t.emplace_back(instance.alphabet[j]);
                }
            }
        }
        cout << endl;
        for (const auto& ch : t) {
            cout << ch;
        }
        cout << "\n------------------------" << endl;
        for (int s = 0; s < instance.numberOfStrings; s++) {
            for (int i = 0; i < instance.stringLength; i++) {
                cout << instance.strings[s][i];
            }
            cout << endl;
        }

        cout << endl;
        cout << "==================================\n";

        
    }
    else {
        model.computeIIS();
        model.write("csp_model_IIS.ilp");
    }

}



