#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

class CTriangle {
private:
    std::vector<int> m_indices;
    std::vector<float> m_lengths;
    float m_radius;

public:
    CTriangle(const std::vector<Eigen::Vector3f>& points, const int index0, const int index1, const int index2);
    ~CTriangle();
    bool operator <(const CTriangle& triangle) const;
    bool operator ==(const CTriangle& triangle) const;
    std::vector<int> getIndices() const;
    std::vector<float> getLengths() const;
    float getRadius() const;
};

#endif