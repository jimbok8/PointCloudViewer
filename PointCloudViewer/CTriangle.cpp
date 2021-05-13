#include "CTriangle.h"

CTriangle::CTriangle(const std::vector<Eigen::Vector3f>& points, const int index0, const int index1, const int index2) {
    m_indices.push_back(index0);
    m_indices.push_back(index1);
    m_indices.push_back(index2);
    std::sort(m_indices.begin(), m_indices.end());

    m_lengths.push_back((points[index0] - points[index1]).norm());
    m_lengths.push_back((points[index1] - points[index2]).norm());
    m_lengths.push_back((points[index2] - points[index0]).norm());

    float a = m_lengths[0];
    float b = m_lengths[1];
    float c = m_lengths[2];
    float p = (a + b + c) / 2.0f;

    m_radius = a * b * c / std::sqrt(4.0f * p * (p - a) * (p - b) * (p - c));
}

CTriangle::~CTriangle() {}

bool CTriangle::operator <(const CTriangle& triangle) const {
    for (int i = 0; i < 3; i++)
        if (m_indices[i] != triangle.m_indices[i])
            return m_indices[i] < triangle.m_indices[i];

    return false;
}

bool CTriangle::operator ==(const CTriangle& triangle) const {
    for (int i = 0; i < 3; i++)
        if (m_indices[i] != triangle.m_indices[i])
            return false;

    return true;
}

std::vector<int> CTriangle::getIndices() const {
    return m_indices;
}

std::vector<float> CTriangle::getLengths() const {
    return m_lengths;
}

float CTriangle::getRadius() const {
    return m_radius;
}