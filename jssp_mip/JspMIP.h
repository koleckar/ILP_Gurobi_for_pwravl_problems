#pragma once

#include "ModelMIP.h"

class JspMIP : public ModelMIP{

public:
	void solveInstance(const char* instancePath, float timeLimit);
};
