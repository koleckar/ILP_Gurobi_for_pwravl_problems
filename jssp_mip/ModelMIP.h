#pragma once

class ModelMIP {

public:
    virtual void solveInstance(const char* instancePath, float timeLimit) = 0;
};


