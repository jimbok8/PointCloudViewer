#include "Point.h"

Point::Point(const Vector3D& position) :
    position(position),
    normal(0.0f, 0.0f, 0.0f) {}

Point::Point(const Vector3D& position, const Vector3D& normal) :
    position(position),
    normal(normal) {}

Point::~Point() {}
