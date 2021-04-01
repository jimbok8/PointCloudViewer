#ifndef POINT_SET_H
#define POINT_SET_H

#include <cfloat>
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>
#include <set>
#include <map>

#include <ANN/ANN.h>
#include <glad/glad.h>

#include "UtilsHelper.h"
#include "Point.h"

class PointSet {
private:
    std::vector<Point> points;
    ANNpointArray pointArray;
    ANNkd_tree* tree;
    unsigned int vao;
    void init();
    void calculateNormals(int k = 50);
    float averageSpacing(int k = 6) const;
    std::vector<std::vector<int>> calculateNeighbors(const std::vector<Point>& points, const float radius) const;
    void selectBasePoint(const std::vector<Point>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const;
    void updateNewPoint(std::vector<Point>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const;

public:
    PointSet();
    PointSet(const std::vector<Point>& points);
    PointSet(const PointSet& pointSet);
    ~PointSet();
    PointSet simplify(const float epsilon) const;
    PointSet resample(float sharpnessAngle, float edgeSensitivity, float neighborRadius, int size) const;
    void render() const;
};

#endif