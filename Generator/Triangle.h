#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <cfloat>
#include <cmath>

#include <glm/glm.hpp>

#include "ConfigHelper.h"
#include "AABB.h"
#include "Ray.h"

class Triangle {
private:
    glm::vec3 p0, p1, p2;
    glm::vec3 center;

public:
    Triangle(const glm::vec3 &p0, const glm::vec3 &p1, const glm::vec3 &p2);
    ~Triangle();
    glm::vec3 getCenter() const;
    AABB aabb() const;
    float trace(const Ray &ray) const;
};

#endif