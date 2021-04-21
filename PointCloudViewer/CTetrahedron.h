#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <vector>

#include <Eigen/Dense>

class CTetrahedron {
private:
    std::vector<int> m_indices;
    std::vector<std::tuple<int, int, int>> m_triangles;

public:
    CTetrahedron(const int i0, const int i1, const int i2, const int i3);
    ~CTetrahedron();
    std::vector<std::tuple<int, int, int>> getTriangles() const;
    bool contain(const Eigen::Vector3f& point) const;
};

#endif