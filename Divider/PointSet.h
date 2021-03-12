#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <fstream>
#include <queue>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "Vertex.h"
#include "Shader.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointNormal;

class PointSet {
private:
    std::vector<Vertex> vertices;
    unsigned int vao;
    std::vector<PointNormal> toPointNormal(std::vector<Vertex>& vertices);
    std::vector<Vertex> fromPointNormal(std::vector<PointNormal>& points);
    void calculateNormals();

public:
    PointSet(std::vector<Vertex>& vertices);
    ~PointSet();
    int size();
    std::vector<Vertex> getVertices();
    std::vector<PointSet> divide(double epsilon, float threshold);
    void render();
};

#endif
