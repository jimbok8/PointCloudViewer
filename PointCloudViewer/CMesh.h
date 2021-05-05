#ifndef MESH_H
#define MESH_H

#include <climits>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <glad/glad.h>

#include "CPoint.h"

class CMesh {
private:
    std::vector<CPoint> m_points;
    std::vector<unsigned int> m_indices;
    unsigned int m_vao;
    void calculateNormals();

public:
    CMesh(const std::vector<CPoint>& points, const std::vector<unsigned int>& indices);
    ~CMesh();
    std::vector<unsigned int> getIndices() const;
    void render() const;
};

#endif