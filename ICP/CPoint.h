#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>

class CPoint {
public:
    Eigen::Vector3f m_position, m_normal, m_color;
    CPoint(const Eigen::Vector3f& position);
    CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& normal);
    CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, const Eigen::Vector3f& color);
    ~CPoint();
};

#endif
