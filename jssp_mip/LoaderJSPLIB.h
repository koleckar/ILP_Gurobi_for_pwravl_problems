#pragma once

#include "JspInstance.h"

#include <string>


class LoaderJSPLIB {

public:
    static JSPLIBInstance loadInstance(const std::string& path);

};
