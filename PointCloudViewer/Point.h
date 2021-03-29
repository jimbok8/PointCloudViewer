#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>

class Point {
public:
    Eigen::Vector3f position, normal;
    Point();
    Point(const Eigen::Vector3f& position);
    Point(const Eigen::Vector3f& position, const Eigen::Vector3f& normal);
    ~Point();
};

#endif
