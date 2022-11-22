#pragma once

#include "CspInstance.h"

class CspReader{
public:
	static CspInstance loadInstance(const char* instancePath);
};

