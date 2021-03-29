#ifndef POINT_SET_H
#define POINT_SET_H

#include <vector>

#include <ANN/ANN.h>
#include <glad/glad.h>

#include "Point.h"

class PointSet {
private:
    std::vector<Point> points;
    ANNpointArray pointArray;
    ANNkd_tree tree;
    unsigned int vao;
    void calculateNormals();

public:
    PointSet(const std::vector<Point>& points);
    ~PointSet();
    void render() const;
};

#endif