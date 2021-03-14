#ifndef POINT_H
#define POINT_H

#include "Vector3D.h"

class Point {
public:
    Vector3D position, normal;
    Point(const Vector3D& position);
    Point(const Vector3D& position, const Vector3D& normal);
    ~Point();
};

#endif
