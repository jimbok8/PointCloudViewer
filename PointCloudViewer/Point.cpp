#include "Point.h"

Point::Point(Vector3D& position) :
    position(position),
    normal(0.0f, 0.0f, 0.0f) {}

Point::Point(Vector3D& position, Vector3D& normal) :
    position(position),
    normal(normal) {}

Point::~Point() {}
