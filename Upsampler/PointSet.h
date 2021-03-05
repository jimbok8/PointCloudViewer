#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <cmath>
#include <fstream>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "Vertex.h"
#include "Shader.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointNormal;
typedef CGAL::Search_traits_3<Kernel> SearchTraits;
typedef CGAL::Orthogonal_k_neighbor_search<SearchTraits> NeighborSearch;
typedef NeighborSearch::Tree Tree;

class PointSet {
private:
	std::vector<Vertex> vertices;
	unsigned int vao;
	std::vector<Point> toPoint(std::vector<Vertex>& vertices);
	std::vector<Vertex> fromPoint(std::vector<Point>& points);
	std::vector<PointNormal> toPointNormal(std::vector<Vertex>& vertices);
	std::vector<Vertex> fromPointNormal(std::vector<PointNormal>& points);
	void calculateNormals();

public:
    PointSet();
	PointSet(std::vector<Vertex>& vertices);
	~PointSet();
	int size();
	PointSet upsample(double sharpnessAngle, double edgeSensitivity, double neighborRadius, int size);
	void render();
};

#endif
