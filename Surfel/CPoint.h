#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>

class CPoint {
public:
    Eigen::Vector3f m_position, m_u, m_v;
    CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& u, const Eigen::Vector3f& v);
    ~CPoint();
};

#endif
