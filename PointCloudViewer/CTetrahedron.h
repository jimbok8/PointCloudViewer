#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

class CTetrahedron {
private:
    std::vector<int> m_indices;
    std::vector<std::tuple<int, int, int>> m_triangles;
    Eigen::Vector3f m_center;
    float m_radius2;

public:
    CTetrahedron(const std::vector<Eigen::Vector3f>& points, const int i0, const int i1, const int i2, const int i3);
    ~CTetrahedron();
    std::vector<std::tuple<int, int, int>> getTriangles() const;
    bool contain(const Eigen::Vector3f& point) const;
    bool boundary(const int size) const;
};

#endif