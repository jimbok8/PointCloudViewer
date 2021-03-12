#ifndef VERTEX_H
#define VERTEX_H

#include <glm/glm.hpp>

struct Vertex {
public:
    glm::vec3 position, normal;
    Vertex(float x, float y, float z);
    Vertex(float x, float y, float z, float nx, float ny, float nz);
    ~Vertex();
};

#endif