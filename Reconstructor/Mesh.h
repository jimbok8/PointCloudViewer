#ifndef MESH_H
#define MESH_H

#include <climits>
#include <numeric>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "Vertex.h"
#include "Shader.h"

class Mesh {
private:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int vao;
    void calculateNormals();

public:
    Mesh();
    Mesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices);
    ~Mesh();
    void render();
};

#endif