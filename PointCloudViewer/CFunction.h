#ifndef FUNCTION_H
#define FUNCTION_H

#include <Eigen/Dense>

class CFunction {
public:
    CFunction();
    ~CFunction();
    virtual float f(const Eigen::Vector3f& x) const = 0;
};

#endif