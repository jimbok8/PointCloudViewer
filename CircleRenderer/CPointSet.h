#ifndef POINT_SET_H
#define POINT_SET_H

#include <cfloat>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <fstream>

#include <Eigen/Dense>
#include <glad/glad.h>

#include "CPoint.h"

class CPointSet {
private:
    std::vector<CPoint> m_points, m_vertices;
    std::vector<unsigned int> m_indices;
    unsigned int m_vao;
    float m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;

public:
    CPointSet(const std::string& path);
    ~CPointSet();
    float getMinX() const;
    float getMaxX() const;
    float scale() const;
    void render() const;
};

#endif