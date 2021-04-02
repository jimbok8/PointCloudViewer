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

#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Surface_mesh.h>

#include "UtilsHelper.h"
#include "CPoint.h"
#include "CMesh.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 CGALPoint;
typedef Kernel::Vector_3 Vector;
typedef std::pair<CGALPoint, Vector> PointNormal;
typedef CGAL::Surface_mesh<CPoint> SurfaceMesh;
typedef SurfaceMesh::Vertex_index VertexIndex;
typedef SurfaceMesh::Face_index FaceIndex;
typedef SurfaceMesh::Halfedge_index HalfedgeIndex;

class CPointSet {
private:
    std::vector<CPoint> m_points;
    ANNpointArray m_pointArray;
    ANNkd_tree* m_tree;
    unsigned int m_vao;
    std::vector<CGALPoint> toPoint(const std::vector<CPoint>& points) const;
    std::vector<CPoint> fromPoint(const std::vector<CGALPoint>& points) const;
    std::vector<PointNormal> toPointNormal(const std::vector<CPoint>& points) const;
    std::vector<CPoint> fromPointNormal(const std::vector<PointNormal>& points) const;
    void calculateNormals(int k = 50);
    float averageSpacing(int k = 6) const;
    std::vector<std::vector<int>> calculateNeighbors(const std::vector<CPoint>& points, const float radius) const;
    void selectBasePoint(const std::vector<CPoint>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const;
    void updateNewPoint(std::vector<CPoint>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const;

public:
    CPointSet(const std::vector<CPoint>& points);
    ~CPointSet();
    CPointSet* simplify(const float epsilon) const;
    CPointSet* resample(float sharpnessAngle, float edgeSensitivity, float neighborRadius, int size) const;
    CPointSet* smooth(const int k) const;
    CMesh* reconstruct(const double maximumFacetLength) const;
    void render() const;
};

#endif