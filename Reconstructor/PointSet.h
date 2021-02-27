#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <fstream>
#include <queue>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include "Vertex.h"
#include "Shader.h"
#include "Mesh.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointNormal;
typedef CGAL::Surface_mesh<Point> SurfaceMesh;
typedef SurfaceMesh::Vertex_index VertexIndex;
typedef SurfaceMesh::Face_index FaceIndex;
typedef SurfaceMesh::Halfedge_index HalfedgeIndex;

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
	PointSet(std::vector<Vertex>& vertices);
	~PointSet();
	int size();
	std::vector<PointSet> divide(double scale, float threshold);
	PointSet removeOutliers(int k, double scale);
	PointSet simplify(double scale);
	PointSet smooth(int k);
	Mesh reconstruct(int type, int resolution);
	void render();
};

#endif
