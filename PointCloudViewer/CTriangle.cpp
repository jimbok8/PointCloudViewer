#include "CTriangle.h"

CTriangle::CTriangle(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
    m_points.push_back(p0);
    m_points.push_back(p1);
    m_points.push_back(p2);
}

CTriangle::~CTriangle() {}

std::vector<Eigen::Vector3f> CTriangle::getPoints() const {
    return m_points;
}