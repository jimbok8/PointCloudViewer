#include "Triangle.h"

Triangle::Triangle(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2) :
p0(p0),
p1(p1),
p2(p2),
center((p0 + p1 + p2) / 3.0f) {}

Triangle::~Triangle() {}

glm::vec3 Triangle::getCenter() const {
    return center;
}

AABB Triangle::aabb() const {
    AABB ans;
    ans.add(p0);
    ans.add(p1);
    ans.add(p2);
    return ans;
}

float Triangle::trace(const Ray &ray) const {
    glm::vec3 o = ray.getOrigin();
    glm::vec3 d = ray.getDirection();
    glm::vec3 e1 = p1 - p0;
    glm::vec3 e2 = p2 - p0;
    glm::vec3 s = o - p0;
    glm::vec3 s1 = glm::cross(s, e1);
    glm::vec3 s2 = glm::cross(d, e2);

    float w = glm::dot(e1, s2);
    if (w < EPSILON)
        return FLT_MAX;
    else {
        float tTemp = glm::dot(e2, s1) / w;
        float u = glm::dot(s, s2) / w;
        float v = glm::dot(d, s1) / w;
        if (tTemp > EPSILON && u >= 0.0f && v >= 0.0f && u + v <= 1.0f)
            return tTemp;
        else
            return FLT_MAX;
    }
}