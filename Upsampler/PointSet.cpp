#include "PointSet.h"

PointSet::PointSet() {}

PointSet::PointSet(std::vector<Vertex> &vertices) {
	this->vertices = vertices;

    calculateSpacing();
    calculateNormals();

	unsigned int vbo;
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &(this->vertices[0]), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

	glBindVertexArray(0);
}

PointSet::~PointSet() {}

std::vector<Point> PointSet::toPoint(std::vector<Vertex>& vertices) {
    std::vector<Point> ans;
    for (Vertex& vertex : vertices)
        ans.push_back(Point(vertex.position.x, vertex.position.y, vertex.position.z));

    return ans;
}

std::vector<Vertex> PointSet::fromPoint(std::vector<Point>& points) {
    std::vector<Vertex> ans;
    for (Point& point : points)
        ans.push_back(Vertex(point.x(), point.y(), point.z()));

    return ans;
}

std::vector<PointNormal> PointSet::toPointNormal(std::vector<Vertex>& vertices) {
    std::vector<PointNormal> ans;
    for (Vertex& vertex : vertices)
        ans.push_back(std::make_pair(Point(vertex.position.x, vertex.position.y, vertex.position.z), Vector(vertex.normal.x, vertex.normal.y, vertex.normal.z)));

    return ans;
}

std::vector<Vertex> PointSet::fromPointNormal(std::vector<PointNormal>& points) {
    std::vector<Vertex> ans;
    for (PointNormal& point : points)
        ans.push_back(Vertex(point.first.x(), point.first.y(), point.first.z(), point.second.x(), point.second.y(), point.second.z()));

    return ans;
}

void PointSet::calculateSpacing() {
    std::vector<Point> points = toPoint(vertices);
    spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 24);
}

void PointSet::calculateNormals() {
    std::vector<PointNormal> points = toPointNormal(vertices);
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, 24, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));

    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].normal = glm::normalize(glm::vec3(points[i].second.x(), points[i].second.y(), points[i].second.z()));
        if (glm::dot(vertices[i].normal, glm::vec3(0.0f, 0.0f, 100.0f) - vertices[i].position) < 0)
               vertices[i].normal = -vertices[i].normal;
    }
}

int PointSet::size() {
    return vertices.size();
}

PointSet PointSet::upsample(int type, double sharpnessAngle, double edgeSensitivity, double neighborRadius, double searchRadius, double upsamplingRadius, double stepSize) {
    std::vector<Vertex> vertices;
    if (type == 0) {
        std::vector<PointNormal> points = toPointNormal(this->vertices);
        CGAL::edge_aware_upsample_point_set<CGAL::Sequential_tag>(points, std::back_inserter(points),
            CGAL::parameters::
            point_map(CGAL::First_of_pair_property_map<PointNormal>()).
            normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
            sharpness_angle(sharpnessAngle).
            edge_sensitivity(edgeSensitivity).
            neighbor_radius(neighborRadius).
            number_of_output_points(points.size() * 4));
        vertices = fromPointNormal(points);
    }
    else if (type == 1) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
        for (Vertex& vertex : this->vertices)
            points->push_back(pcl::PointXYZ(vertex.position.x, vertex.position.y, vertex.position.z));
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setComputeNormals(true);
        mls.setInputCloud(points);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(searchRadius);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(upsamplingRadius);
        mls.setUpsamplingStepSize(stepSize);
        pcl::PointCloud<pcl::PointXYZ>::Ptr upsample(new pcl::PointCloud<pcl::PointXYZ>);
        mls.process(*upsample);
        for (pcl::PointXYZ& point : *upsample)
            vertices.push_back(Vertex(point.x, point.y, point.z));
    }

    return PointSet(vertices);
}

void PointSet::render() {
	glPointSize(2);
	glBindVertexArray(vao);
	glDrawArrays(GL_POINTS, 0, vertices.size());
	glBindVertexArray(0);
}