#pragma once

#include <string>


struct CspInstance {
	int alphabetSize;
	char *alphabet;

	int stringLength;
	int numberOfStrings;
	std::string *strings;
};