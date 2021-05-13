#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

#include "CTriangle.h"

class CTetrahedron {
private:
    std::vector<CTriangle> m_triangles;
    Eigen::Vector3f m_center;
    float m_radius2;
    void calculateCircumcircle(const std::vector<Eigen::Vector3f>& points, const int index0, const int index1, const int index2, const int index3);

public:
    CTetrahedron(const std::vector<Eigen::Vector3f>& points, const int index0, const int index1, const int index2, const int index3);
    CTetrahedron(const std::vector<Eigen::Vector3f>& points, const int index, const CTriangle& triangle);
    ~CTetrahedron();
    std::vector<CTriangle> getTriangles() const;
    bool contain(const Eigen::Vector3f& point) const;
};

#endif