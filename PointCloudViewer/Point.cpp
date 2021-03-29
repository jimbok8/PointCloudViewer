#include "Point.h"

Point::Point() :
    position(0.0f, 0.0f, 0.0f),
    normal(0.0f, 0.0f, 0.0f) {}

Point::Point(const Eigen::Vector3f& position) :
    position(position),
    normal(0.0f, 0.0f, 0.0f) {}

Point::Point(const Eigen::Vector3f& position, const Eigen::Vector3f& normal) :
    position(position),
    normal(normal) {}

Point::~Point() {}
