#ifndef POINT_SET_H
#define POINT_SET_H

#include <cfloat>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <fstream>

#include <Eigen/Dense>
#include <ANN/ANN.h>
#include <glad/glad.h>

#include "CPoint.h"
#include "CSimplifyParameter.h"
#include "CSmoothParameter.h"

class CPointSet {
private:
    std::vector<CPoint> m_points;
    ANNpointArray m_pointArray;
    ANNkd_tree* m_tree;
    unsigned int m_vao;
    void bind();
    void initialize(bool normalFlag);
    std::vector<std::vector<int>> calculateKNeighbors(int k) const;
    void calculateNormals(const int k = 50);
    Eigen::Vector3f centroid() const;
    Eigen::Vector3f nearest(const Eigen::Vector3f& point) const;

public:
    CPointSet(const std::string& path);
    CPointSet(const std::vector<CPoint>& points, bool normalFlag);
    ~CPointSet();
    std::vector<CPoint> getPoints() const;
    int size() const;
    float scale() const;
    float averageSpacing(const int k = 6) const;
    CPointSet* simplify(const CSimplifyParameter& parameter, bool normalFlag) const;
    CPointSet* smooth(const CSmoothParameter& parameter, bool normalFlag) const;
    void registrate(const CPointSet* target, Eigen::Matrix3f& rotate, Eigen::Vector3f& translate) const;
    CPointSet* combine(const CPointSet* set, bool normalFlag) const;
    void combine(const CPointSet* set, const Eigen::Matrix3f& rotate, const Eigen::Vector3f& translate);
    void render() const;
    void save(const std::string& path) const;
};

#endif