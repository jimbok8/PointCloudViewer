#include "CTetrahedron.h"

CTetrahedron::CTetrahedron(const std::vector<Eigen::Vector3f>& points, const int i0, const int i1, const int i2, const int i3) {
    m_indices.push_back(i0);
    m_indices.push_back(i1);
    m_indices.push_back(i2);
    m_indices.push_back(i3);
    std::sort(m_indices.begin(), m_indices.end());

    m_triangles.push_back(std::make_tuple(m_indices[0], m_indices[1], m_indices[2]));
    m_triangles.push_back(std::make_tuple(m_indices[0], m_indices[1], m_indices[3]));
    m_triangles.push_back(std::make_tuple(m_indices[0], m_indices[2], m_indices[3]));
    m_triangles.push_back(std::make_tuple(m_indices[1], m_indices[2], m_indices[3]));

    Eigen::Vector3f v0 = points[m_indices[0]];
    Eigen::Vector3f v1 = points[m_indices[1]];
    Eigen::Vector3f v2 = points[m_indices[2]];
    Eigen::Vector3f v3 = points[m_indices[3]];

    Eigen::Vector3f u1 = v1 - v0;
    Eigen::Vector3f u2 = v2 - v0;
    Eigen::Vector3f u3 = v3 - v0;

    m_center = v0 + (u1.squaredNorm() * u2.cross(u3) + u2.squaredNorm() * u3.cross(u1) + u3.squaredNorm() * u1.cross(u2)) / (2.0 * u1.dot(u2.cross(u3)));
    m_radius2 = (m_center - v0).squaredNorm();
}

CTetrahedron::~CTetrahedron() {}

std::vector<std::tuple<int, int, int>> CTetrahedron::getTriangles() const {
    return m_triangles;
}

bool CTetrahedron::contain(const Eigen::Vector3f& point) const {
    return (point - m_center).squaredNorm() <= m_radius2;
}