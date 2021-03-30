#ifndef POINT_SET_H
#define POINT_SET_H

#include <cfloat>
#include <algorithm>
#include <vector>
#include <queue>

#include <ANN/ANN.h>
#include <glad/glad.h>

#include "Point.h"

class PointSet {
private:
    std::vector<Point> points;
    ANNpointArray pointArray;
    ANNkd_tree* tree;
    unsigned int vao;
    void init();
    void calculateNormals(const int k = 50);

public:
    PointSet(const std::vector<Point>& points);
    PointSet(const PointSet& pointSet);
    ~PointSet();
    PointSet simplify(float epsilon);
    void render() const;
};

#endif