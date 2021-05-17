#include "Vertex.h"

Vertex::Vertex(const glm::vec3& position) :
	position(position),
	normal(0.0f, 0.0f, 0.0f) {}

Vertex::Vertex(const glm::vec3& position, const glm::vec3& normal) :
	position(position),
	normal(normal) {}

Vertex::~Vertex() {}