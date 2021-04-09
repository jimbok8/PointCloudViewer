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
#include "CPoint.h"
#include "CMesh.h"
#include "CSimplifyParameter.h"
#include "CResampleParameter.h"

class CPointSet {
private:
    std::vector<CPoint> m_points;
    ANNpointArray m_pointArray;
    ANNkd_tree* m_tree;
    unsigned int m_vao;
    void calculateNormals(int k = 50);
    float averageSpacing(int k = 6) const;
    std::vector<std::vector<int>> calculateNeighbors(const std::vector<CPoint>& points, const float radius) const;
    void selectBasePoint(const std::vector<CPoint>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const;
    void updateNewPoint(std::vector<CPoint>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const;

public:
    CPointSet(const std::vector<CPoint>& points);
    ~CPointSet();
    std::vector<CPoint> getPoints() const;
    CPointSet* simplify(const CSimplifyParameter& parameter) const;
    CPointSet* resample(const CResampleParameter& parameter) const;
    CPointSet* smooth(const int k) const;
    CMesh* reconstruct(const double maximumFacetLength) const;
    void render() const;
};

#endif