#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <vector>

#include <Eigen/Dense>

class CTriangle {
private:
    std::vector<Eigen::Vector3f> m_points;

public:
    CTriangle(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2);
    ~CTriangle();
    std::vector<Eigen::Vector3f> getPoints() const;
};

#endif