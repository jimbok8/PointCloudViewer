#ifndef AABB_H
#define AABB_H

#include <cfloat>
#include <cmath>
#include <algorithm>

#include <glm/glm.hpp>

#include "ConfigHelper.h"
#include "Ray.h"

class AABB {
private:
    float x0, x1, y0, y1, z0, z1;

public:
    AABB();
    ~AABB();
    void add(const glm::vec3 &point);
    void combine(const AABB &aabb);
    float rangeX() const;
    float rangeY() const;
    float rangeZ() const;
    bool trace(const Ray &ray) const;
};

#endif