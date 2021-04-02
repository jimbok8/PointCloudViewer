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
#include "Point.h"
#include "Mesh.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 CGALPoint;
typedef Kernel::Vector_3 Vector;
typedef std::pair<CGALPoint, Vector> PointNormal;
typedef CGAL::Surface_mesh<Point> SurfaceMesh;
typedef SurfaceMesh::Vertex_index VertexIndex;
typedef SurfaceMesh::Face_index FaceIndex;
typedef SurfaceMesh::Halfedge_index HalfedgeIndex;

class PointSet {
private:
    std::vector<Point> points;
    ANNpointArray pointArray;
    ANNkd_tree* tree;
    unsigned int vao;
    std::vector<CGALPoint> toPoint(const std::vector<Point>& points) const;
    std::vector<Point> fromPoint(const std::vector<CGALPoint>& points) const;
    std::vector<PointNormal> toPointNormal(const std::vector<Point>& points) const;
    std::vector<Point> fromPointNormal(const std::vector<PointNormal>& points) const;
    void calculateNormals(int k = 50);
    float averageSpacing(int k = 6) const;
    std::vector<std::vector<int>> calculateNeighbors(const std::vector<Point>& points, const float radius) const;
    void selectBasePoint(const std::vector<Point>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const;
    void updateNewPoint(std::vector<Point>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const;

public:
    PointSet(const std::vector<Point>& points);
    ~PointSet();
    PointSet* simplify(const float epsilon) const;
    PointSet* resample(float sharpnessAngle, float edgeSensitivity, float neighborRadius, int size) const;
    PointSet* smooth(const int k) const;
    Mesh* reconstruct(const double maximumFacetLength) const;
    void render() const;
};

#endif