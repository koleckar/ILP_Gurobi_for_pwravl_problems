#include "GrMIP.h"
#include "GrInstance.h"
#include "GrReader.h"

#include "gurobi_c++.h"

#include <numeric>
#include <algorithm>
#include <string>

using std::string;
using std::cout;
using std::endl;


void GrMIP::solveInstance(const char* instancePath, float timeLimit) {
// MIP model as in 74/210 Dias, Souza: http://bsb2007.inf.puc-rio.br/poster_proceedings.pdf, pg.78

    GrInstance instance;
    // int n = instance.permutationLength;

    int perm1[] = { 0, 1, 2, 3, 5, 6, 4 };
    int perm2[] = { 0, 1, 2, 5, 6, 4, 3 };
    int n = 7;

    // ------ Gurobi model. ---------------
    GRBEnv* env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.set(GRB_StringAttr_ModelName, "Genome Rearrangement problem MIP model.");
    model.set(GRB_DoubleParam_TimeLimit, timeLimit);

    // ------ Variables. ---------------

    // B: k [0, n] | t: k [1, n] 
    GRBVar*** B = new GRBVar ** [n]; // i = len of perm, k-th operation, has value j
    for (int i = 0; i < n; i++) {
        B[i] = new  GRBVar * [n];
        for (int j = 0; j < n; j++) {
            B[i][j] = new  GRBVar[n];
            for (int k = 0; k < n; k++) {
                string name = "B" + std::to_string(i) + std::to_string(j) + std::to_string(k);
                B[i][j][k] = model.addVar(0, 1, 0, GRB_BINARY, name);
            }
        }
    }

    // So k_0 is identity? 
    GRBVar**** T = new GRBVar * * *[n + 1]; // t[a,b,c,k]   a,b,c indexed from 0 not from 1 as in paper
    for (int a = 0; a < n + 1; a++) {
        T[a] = new  GRBVar * *[n + 1];
        for (int b = a + 1; b < n + 1; b++) {
            T[a][b] = new  GRBVar * [n + 1];
            for (int c = b + 1; c < n + 1; c++) {
                T[a][b][c] = new  GRBVar[n];
                for (int k = 0; k < n; k++) { // k indexed from 0
                    string name = "T" + std::to_string(a) + std::to_string(b) + std::to_string(c) + std::to_string(k);
                    T[a][b][c][k] = model.addVar(0, 1, 0, GRB_BINARY, name);
                }
            }
        }
    }

    GRBVar* t = new GRBVar[n]; // 't[k]' tells whether kth transposition operation has modified the permutation.
    for (int k = 0; k < n; k++) {
        string name = "t" + std::to_string(k);
        t[k] = model.addVar(0, 1, 1, GRB_BINARY, name);
    }


    // ------ Constraints. ---------------
    GRBLinExpr expr;
    GRBLinExpr expr2;

    // (1)
    for (int i = 0; i < n; i++) {
        model.addConstr(B[i][perm1[i]][0] == 1);
    }

    // (2)
    for (int i = 0; i < n; i++) {
        model.addConstr(B[i][perm2[i]][n - 1] == 1);
    }

    // (3)
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < n; k++) {
            expr = 0;
            for (int j = 0; j < n; j++) {
                expr += B[i][j][k];
            }
            model.addConstr(expr == 1);
        }   
    }

    // (4)
    for (int j = 0; j < n; j++) {
        for (int k = 0; k < n; k++) {
            expr = 0;
            for (int i = 0; i < n; i++) {
                expr += B[i][j][k];
            }
            model.addConstr(expr == 1);
        }
    }

    // (5)
    for (int k = 1; k < n; k++) {
        model.addConstr(t[k] <= t[k - 1]);
    }

    // (6)
    for (int k = 1; k < n; k++) {
        expr = 0;
        for (int a = 0; a < n - 1; a++) {  // a, b, c here indexed from 0, not from 1 as in paper
            for (int b = a + 1; b < n; b++) {
                for (int c = b + 1; c < n + 1; c++) {
                    expr += T[a][b][c][k];
                }
            }
        }
        model.addConstr(expr <= t[k]);
    }

    // (7)
    for (int k = 1; k < n; k++) {
        for (int i = 0; i < n; i++) { // i,j < n not i,j <= n as in paper, because we index from 0 not from 1
            for (int j = 0; j < n; j++) {
                expr = 0;
                expr2 = 0;
                for (int a = i + 1; a < n - 1; a++) {
                    for (int b = a + 1; b < n; b++) {
                        for (int c = b + 1; c < n + 1; c++) {
                            expr += T[a][b][c][k];
                        }
                    }
                }
                for (int a = 0; a < n - 1; a++) {
                    for (int b = a + 1; b < n; b++) {
                        for (int c = b + 1; c < i; c++) {
                            expr2 += T[a][b][c][k];
                        }
                    }
                }
                model.addConstr(expr + expr2 + (1 - t[k]) + B[i][j][k - 1] - B[i][j][k] <= 1);
            }
        }
    }

    // (8)
    for (int k = 1; k < n; k++) {
        for (int j = 0; j < n; j++) { // j < n not j <= n as in paper, because we index from 0 not from 1
            for (int a = 0; a < n + 1; a++) {
                for (int b = a + 1; b < n + 1; b++) {
                    for (int c = b + 1; c < n + 1; c++) {
                        for (int i = a; i < a + c - b; i++) {
                            model.addConstr(T[a][b][c][k] + B[b - a + i][j][k - 1] - B[i][j][k] <= 1);
                        }
                    }
                }
            }
        }
    }

    // (9)
    for (int k = 1; k < n; k++) {
        for (int j = 0; j < n; j++) { // j < n not j <= n as in paper, because we index from 0 not from 1
            for (int a = 0; a < n + 1; a++) {
                for (int b = a + 1; b < n + 1; b++) {
                    for (int c = b + 1; c < n + 1; c++) {
                        for (int i = a + c - b; i < c; i++) {
                            model.addConstr(T[a][b][c][k] + B[b - c + i][j][k - 1] - B[i][j][k] <= 1);
                        }
                    }
                }
            }
        }
    }


    //------- Solve the model. -----------
    model.optimize();


    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
        cout << "\n==================================" << endl;
        cout << "minimal number of transpositions: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (B[i][j][k].get(GRB_DoubleAttr_X) == 1) {
                        cout << j << " ";
                    }
                }
            }
            cout << endl;
        }
        cout << "----------------------------------\n";
        for (int k = 0; k < n; k++) {
            cout << t[k].get(GRB_DoubleAttr_X) << " ";
        }
        cout << endl;
        cout << "==================================\n";
    }
    else {
        model.computeIIS();
        model.write("gr_model_IIS.ilp");
    }

}