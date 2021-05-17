#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <cmath>
#include <fstream>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>

#include "Vertex.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointNormal;

class PointSet {
private:
    std::vector<Vertex> vertices;
    unsigned int vao;

public:
    PointSet();
    PointSet(std::vector<Vertex>& vertices);
    ~PointSet();
    std::vector<Vertex> getVertices() const;
    void render();
};

#endif
