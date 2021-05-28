#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>

class CPoint {
public:
    Eigen::Vector3f m_position, m_normal;
    CPoint();
    CPoint(const Eigen::Vector3f& position);
    ~CPoint();
};

#endif
