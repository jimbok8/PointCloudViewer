#ifndef POINT_H
#define POINT_H

#include "Vector3D.h"

class Point {
public:
    Vector3D position, normal;
    Point(Vector3D& position);
    Point(Vector3D& position, Vector3D& normal);
    ~Point();
};

#endif
