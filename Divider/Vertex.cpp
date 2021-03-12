#include "Vertex.h"

Vertex::Vertex(float x, float y, float z) {
    this->position = glm::vec3(x, y, z);
    this->normal = glm::vec3(0.0f, 0.0f, 0.0f);
}

Vertex::Vertex(float x, float y, float z, float nx, float ny, float nz) {
    this->position = glm::vec3(x, y, z);
    this->normal = glm::vec3(nx, ny, nz);
}

Vertex::~Vertex() {}