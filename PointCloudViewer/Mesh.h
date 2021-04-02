#ifndef MESH_H
#define MESH_H

#include <climits>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <glad/glad.h>

#include "Point.h"

class Mesh {
private:
    std::vector<Point> points;
    std::vector<unsigned int> indices;
    unsigned int vao;
    void calculateNormals();

public:
    Mesh(const std::vector<Point>& points, const std::vector<unsigned int>& indices);
    ~Mesh();
    void render() const;
};

#endif