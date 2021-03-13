#ifndef POINT_SET_H
#define POINT_SET_H

#include <vector>

#include <glad/glad.h>

#include "Point.h"

class PointSet {
private:
    std::vector<Point> points;
    unsigned int vao;

public:
    PointSet(std::vector<Point> points);
    ~PointSet();
    void render() const;
};

#endif