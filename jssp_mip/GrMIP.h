#pragma once

#include "ModelMIP.h"


class GrMIP : public ModelMIP{
public:
	void solveInstance(const char* instancePath, float timeLimit);
};

