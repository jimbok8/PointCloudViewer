#include "Vertex.h"

Vertex::Vertex(const glm::vec3& position, const glm::vec3& normal) :
	position(position),
	normal(normal) {}

Vertex::~Vertex() {}