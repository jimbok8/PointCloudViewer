#ifndef POINT_SET_H
#define POINT_SET_H

#include <cfloat>
#include <cmath>
#include <algorithm>
#include <vector>
#include <list>
#include <queue>
#include <set>
#include <map>

#include <Eigen/Dense>
#include <ANN/ANN.h>
#include <glad/glad.h>

#include "CPoint.h"
#include "CMesh.h"
#include "CTriangle.h"
#include "CTetrahedron.h"
#include "CCandidate.h"
#include "CMeshBoundary.h"
#include "CSimplifyParameter.h"
#include "CResampleParameter.h"
#include "CSmoothParameter.h"
#include "CReconstructParameter.h"

class CPointSet {
private:
    const float ALPHA = acos(-1.0f) * 5.0f / 6.0f;
    std::vector<CPoint> m_points;
    ANNpointArray m_pointArray;
    ANNkd_tree* m_tree;
    unsigned int m_vao;
    std::vector<std::vector<int>> calculateKNeighbors(int k) const;
    std::vector<std::vector<int>> calculateRadiusNeighbors(const float radius) const;
    void calculateNormals(const int k = 50);
    float averageSpacing(const int k = 6) const;
    std::vector<std::vector<int>> calculateRadiusNeighbors(const std::vector<CPoint>& points, const float radius) const;
    void selectBasePoint(const std::vector<CPoint>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const;
    void updateNewPoint(std::vector<CPoint>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const;
    Eigen::Vector3f calculateNormal(const std::vector<Eigen::Vector3f>& points, const int p0, const int p1, const int p2) const;
    CCandidate calculateCandidate(const std::vector<Eigen::Vector3f>& points, const std::vector<float>& radii, const std::vector<bool>& flag, const std::vector<std::pair<int, int>>& candidates, const int source, const int target, const Eigen::Vector3f& normal) const;
    void addEdge(const std::vector<Eigen::Vector3f>& points, std::map<std::pair<int, int>, Eigen::Vector3f>& edges, std::priority_queue<CCandidate>& heap, const std::vector<float>& radii, const std::vector<bool>& flag, const std::vector<std::pair<int, int>>& candidates, const int source, const int target, const Eigen::Vector3f& normal) const;

public:
    CPointSet(const std::vector<CPoint>& points);
    ~CPointSet();
    std::vector<CPoint> getPoints() const;
    CPointSet* simplify(const CSimplifyParameter& parameter) const;
    CPointSet* resample(const CResampleParameter& parameter) const;
    CPointSet* smooth(const CSmoothParameter& parameter) const;
    CMesh* reconstruct(const CReconstructParameter& parameter) const;
    void render() const;
};

#endif