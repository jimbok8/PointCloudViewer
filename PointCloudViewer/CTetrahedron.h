#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <vector>

#include <Eigen/Dense>

#include "CTriangle.h"

class CTetrahedron {
private:
    std::vector<Eigen::Vector3f> m_points;
    std::vector<CTriangle> m_triangles;

public:
    CTetrahedron(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);
    ~CTetrahedron();
    std::vector<CTriangle> getTriangles() const;
    bool contain(const Eigen::Vector3f& point) const;
};

#endif