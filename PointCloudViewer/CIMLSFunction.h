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
    float m_radius2, m_epsilon2;
    ANNpointArray m_pointArray;
    ANNkd_tree* m_tree;
    std::vector<int> m_num;

public:
    CIMLSFunction(const std::vector<CPoint>& points, const float radius, const float epsilon);
    ~CIMLSFunction();
    float f(const Eigen::Vector3f& x) const override;
};

#endif