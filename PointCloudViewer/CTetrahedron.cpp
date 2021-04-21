#include "CTetrahedron.h"

CTetrahedron::CTetrahedron(const int i0, const int i1, const int i2, const int i3) {
    m_indices.push_back(i0);
    m_indices.push_back(i1);
    m_indices.push_back(i2);
    m_indices.push_back(i3);

    m_triangles.push_back(std::make_tuple(i0, i1, i2));
    m_triangles.push_back(std::make_tuple(i0, i1, i3));
    m_triangles.push_back(std::make_tuple(i0, i2, i3));
    m_triangles.push_back(std::make_tuple(i1, i2, i3));
}

CTetrahedron::~CTetrahedron() {}

std::vector<std::tuple<int, int, int>> CTetrahedron::getTriangles() const {
    return m_triangles;
}

bool CTetrahedron::contain(const Eigen::Vector3f& point) const {

}