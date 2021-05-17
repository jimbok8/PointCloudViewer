#ifndef RAY_H
#define RAY_H

#include <glm/glm.hpp>

class Ray {
private:
    glm::vec3 origin, direction;

public:
    Ray(const glm::vec3 &origin, const glm::vec3&direction);
    ~Ray();
    glm::vec3 getOrigin() const;
    glm::vec3 getDirection() const;
    glm::vec3 point(const float t) const;
};

#endif