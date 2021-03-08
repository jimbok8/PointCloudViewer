#include "PointSet.h"

PointSet::PointSet() {}

PointSet::PointSet(std::vector<Vertex> &vertices) {
	this->vertices = vertices;

    bool flag = false;
    for (Vertex& vertex : this->vertices)
        if (glm::length(vertex.normal) < 1e-5f)
            flag = true;
    if (flag)
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

void PointSet::calculateNormals() {
    std::vector<PointNormal> points = toPointNormal(vertices);
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, 24, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));

    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].normal = glm::normalize(glm::vec3(points[i].second.x(), points[i].second.y(), points[i].second.z()));
        /*if (glm::dot(vertices[i].normal, glm::vec3(0.0f, 0.0f, 0.0f) - vertices[i].position) < 0)
               vertices[i].normal = -vertices[i].normal;*/
    }
}

int PointSet::size() {
    return vertices.size();
}

std::vector<PointNormal> PointSet::filter(std::vector<PointNormal>& points, float threshold) {
    Vector avg(0, 0, 0);
    for (PointNormal& point : points)
        avg += Vector(point.first.x(), point.first.y(), point.first.z());
    avg /= points.size();
    
    Eigen::MatrixXf X(3, points.size());
    for (int i = 0; i < points.size(); i++) {
        X(0, i) = points[i].first.x() - avg.x();
        X(1, i) = points[i].first.y() - avg.y();
        X(2, i) = points[i].first.z() - avg.z();
    }
    Eigen::Matrix3f C = X * X.transpose() / points.size();
    
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> solver(C);
    Eigen::Vector3f values = solver.eigenvalues();
    Eigen::Matrix3f vectors = solver.eigenvectors();

    Eigen::MatrixXf W(3, 2);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2; j++)
            W(i, j) = vectors(i, j + 1);

    Eigen::MatrixXf Y(2, points.size());
    Y = W.transpose() * X;
    std::vector<PointNormal> ans;
    for (int i = 0; i < points.size(); i++)
        if (Y(0, i) * Y(0, i) + Y(1, i) * Y(1, i) < threshold)
            ans.push_back(points[i]);
    return ans;
}

PointSet PointSet::upsample(int k, float threshold, double sharpnessAngle, double edgeSensitivity, double neighborRadius, int size) {
    std::vector<PointNormal> points = toPointNormal(this->vertices);
    Tree tree(points.begin(), points.end());
    std::vector<Vertex> vertices;
    for (Vertex& vertex : this->vertices) {
        NeighborSearch search(tree, Point(vertex.position.x, vertex.position.y, vertex.position.z), k);
        std::vector<PointNormal> pointsTemp;
        float avg = 0;
        for (NeighborSearch::iterator iter = search.begin(); iter != search.end(); iter++) {
            pointsTemp.push_back(iter->first);
            avg += std::sqrt(iter->second);
        }
        avg /= k;
        pointsTemp = filter(pointsTemp, threshold);
        CGAL::edge_aware_upsample_point_set<CGAL::Parallel_if_available_tag>(pointsTemp, std::back_inserter(pointsTemp),
            CGAL::parameters::
            point_map(CGAL::First_of_pair_property_map<PointNormal>()).
            normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
            sharpness_angle(sharpnessAngle).
            edge_sensitivity(edgeSensitivity).
            neighbor_radius(avg).
            number_of_output_points((int)(size * avg * avg)));
        std::vector<Vertex> verticesTemp = fromPointNormal(pointsTemp);
        vertices.insert(vertices.end(), verticesTemp.begin(), verticesTemp.end());
    }

    /*std::vector<PointNormal> points = toPointNormal(this->vertices);
    CGAL::edge_aware_upsample_point_set<CGAL::Parallel_if_available_tag>(points, std::back_inserter(points),
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
        sharpness_angle(sharpnessAngle).
        edge_sensitivity(edgeSensitivity).
        neighbor_radius(neighborRadius).
        number_of_output_points(this->vertices.size() * 4));
    std::vector<Vertex> vertices = fromPointNormal(points);*/

    return PointSet(vertices);
}

void PointSet::render() {
	glPointSize(2);
	glBindVertexArray(vao);
	glDrawArrays(GL_POINTS, 0, vertices.size());
	glBindVertexArray(0);
}