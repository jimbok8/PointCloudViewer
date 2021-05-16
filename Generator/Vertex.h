#ifndef VERTEX_H
#define VERTEX_H

#include <glm/glm.hpp>

struct Vertex {
public:
    glm::vec3 position, normal;
    Vertex(const glm::vec3& position, const glm::vec3& normal);
    ~Vertex();
};

#endif