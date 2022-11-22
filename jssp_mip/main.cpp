#pragma once

#include "JspMIP.h"
#include "VrptwMIP.h"
#include "CspMIP.h"
#include "GrMIP.h"


int main() {
	float timeLimit = 3000.0;

	//JspMIP model1;
	//model1.solveInstance("datasets/JSPLIB/abz5.txt", timeLimit);

	//VrptwMIP model2;
	//model2.solveInstance("datasets/VRPREP/solomon-1987-c1/C109_025.xml", timeLimit);

	//CspMIP model3;
	//model3.solveInstance("", timeLimit);

	 GrMIP model4;
	 model4.solveInstance("", timeLimit);


	return 0;
}