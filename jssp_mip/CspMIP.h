#pragma once

#include "ModelMIP.h"

class CspMIP : public ModelMIP{
public:
	void solveInstance(const char* instancePath, float timeLimit);
};

