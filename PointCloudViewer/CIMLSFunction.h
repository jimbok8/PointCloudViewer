#ifndef IMLS_FUNCTION_H
#define IMLS_FUNCTION_H

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <ANN/ANN.h>

#include "CPoint.h"
#include "CFunction.h"

class CIMLSFunction : public CFunction {
private:
    std::vector<CPoint> m_points;
    float m_epsilon2;
    std::vector<int> m_num;

public:
    CIMLSFunction(const std::vector<CPoint>& points, const float epsilon);
    ~CIMLSFunction();
    float f(const Eigen::Vector3f& x) const override;
};

#endif