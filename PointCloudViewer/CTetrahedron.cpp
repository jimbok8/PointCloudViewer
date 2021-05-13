#include "CTetrahedron.h"

CTetrahedron::CTetrahedron(const std::vector<Eigen::Vector3f>& points, const int index0, const int index1, const int index2, const int index3) {
    m_triangles.push_back(CTriangle(points, index0, index1, index2));
    m_triangles.push_back(CTriangle(points, index0, index1, index3));
    m_triangles.push_back(CTriangle(points, index0, index2, index3));
    m_triangles.push_back(CTriangle(points, index1, index2, index3));

    calculateCircumcircle(points, index0, index1, index2, index3);
}

CTetrahedron::CTetrahedron(const std::vector<Eigen::Vector3f>& points, const int index, const CTriangle& triangle) {
    std::vector<int> indices = triangle.getIndices();

    m_triangles.push_back(triangle);
    m_triangles.push_back(CTriangle(points, index, indices[0], indices[1]));
    m_triangles.push_back(CTriangle(points, index, indices[0], indices[2]));
    m_triangles.push_back(CTriangle(points, index, indices[1], indices[2]));

    calculateCircumcircle(points, index, indices[0], indices[1], indices[2]);
}

CTetrahedron::~CTetrahedron() {}

void CTetrahedron::calculateCircumcircle(const std::vector<Eigen::Vector3f>& points, const int index0, const int index1, const int index2, const int index3) {
    Eigen::Vector3f v0 = points[index0];
    Eigen::Vector3f v1 = points[index1];
    Eigen::Vector3f v2 = points[index2];
    Eigen::Vector3f v3 = points[index3];

    Eigen::Vector3f u1 = v1 - v0;
    Eigen::Vector3f u2 = v2 - v0;
    Eigen::Vector3f u3 = v3 - v0;

    m_center = v0 + (u1.squaredNorm() * u2.cross(u3) + u2.squaredNorm() * u3.cross(u1) + u3.squaredNorm() * u1.cross(u2)) / (2.0 * u1.dot(u2.cross(u3)));
    m_radius2 = (m_center - v0).squaredNorm();
}

std::vector<CTriangle> CTetrahedron::getTriangles() const {
    return m_triangles;
}

bool CTetrahedron::contain(const Eigen::Vector3f& point) const {
    return (point - m_center).squaredNorm() <= m_radius2;
}