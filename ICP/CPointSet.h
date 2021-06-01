#ifndef POINT_SET_H
#define POINT_SET_H

#include <vector>

#include <glad/glad.h>

#include "CPoint.h"

class CPointSet {
private:
    std::vector<CPoint> m_points0, m_points1;
    std::vector<unsigned int> m_indices;
    unsigned int m_vao0, m_vao1;
public:
    CPointSet(const std::vector<CPoint>& points0, const std::vector<CPoint>& points1, const std::vector<int>& indices);
    ~CPointSet();
    void render(const bool flag) const;
};

#endif