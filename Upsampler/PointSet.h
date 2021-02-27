#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <fstream>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/edge_aware_upsample_point_set.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

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
	double spacing;
	std::vector<Point> toPoint(std::vector<Vertex>& vertices);
	std::vector<Vertex> fromPoint(std::vector<Point>& points);
	std::vector<PointNormal> toPointNormal(std::vector<Vertex>& vertices);
	std::vector<Vertex> fromPointNormal(std::vector<PointNormal>& points);
	void calculateSpacing();
	void calculateNormals();

public:
    PointSet();
	PointSet(std::vector<Vertex>& vertices);
	~PointSet();
	int size();
	PointSet upsample(int type, double sharpnessAngle, double edgeSensitivity, double neighborRadius, double searchRadius, double upsamplingRadius, double stepSize);
	void render();
};

#endif
