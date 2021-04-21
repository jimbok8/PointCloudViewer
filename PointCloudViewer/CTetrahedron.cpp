#include "CTetrahedron.h"

CTetrahedron::CTetrahedron(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3) {
    m_points.push_back(p0);
    m_points.push_back(p1);
    m_points.push_back(p2);
    m_points.push_back(p3);

    m_triangles.push_back(CTriangle(p0, p1, p2));
    m_triangles.push_back(CTriangle(p0, p1, p3));
    m_triangles.push_back(CTriangle(p0, p2, p3));
    m_triangles.push_back(CTriangle(p1, p2, p3));
}

CTetrahedron::~CTetrahedron() {}

std::vector<CTriangle> CTetrahedron::getTriangles() const {
    return m_triangles;
}

bool CTetrahedron::contain(const Eigen::Vector3f& point) const {

}