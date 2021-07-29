#ifndef SPHERE_FUNCTION_H
#define SPHERE_FUNCTION_H

#include <Eigen/Dense>

#include "CFunction.h"

class CSphereFunction : public CFunction {
private:
    Eigen::Vector3f m_center;
    float m_radius2;

public:
    CSphereFunction(const Eigen::Vector3f& center, const float radius);
    ~CSphereFunction();
    float f(const Eigen::Vector3f& x) const override;
};

#endif