#include "PointSet.h"

PointSet::PointSet() {}

PointSet::PointSet(std::vector<Vertex> &vertices) {
    this->vertices = vertices;
    
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
    CGAL::mst_orient_normals(points, 24, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));

    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].normal = glm::normalize(glm::vec3(points[i].second.x(), points[i].second.y(), points[i].second.z()));
        /*if (glm::dot(vertices[i].normal, glm::vec3(0.0f, 0.0f, 0.0f) - vertices[i].position) < 0)
               vertices[i].normal = -vertices[i].normal;*/
    }
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

int PointSet::size() {
    return vertices.size();
}

PointSet PointSet::simplify(double epsilon) {
    std::vector<Point> points = toPoint(vertices);
    int t = points.size();
    points.erase(CGAL::grid_simplify_point_set(points, epsilon), points.end());
    std::vector<Point>(points).swap(points);
    std::cout << t - points.size() << " point(s) removed after simplification." << std::endl;

    std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

PointSet PointSet::upsample(double sharpnessAngle, double edgeSensitivity, double neighborRadius, int size) {
    //std::vector<PointNormal> points = toPointNormal(this->vertices);
    //Tree tree(points.begin(), points.end());
    //std::vector<Vertex> vertices;
    //for (Vertex& vertex : this->vertices) {
    //    NeighborSearch search(tree, Point(vertex.position.x, vertex.position.y, vertex.position.z), k);
    //    std::vector<PointNormal> pointsTemp;
    //    float avg = 0;
    //    for (NeighborSearch::iterator iter = search.begin(); iter != search.end(); iter++) {
    //        pointsTemp.push_back(iter->first);
    //        avg += std::sqrt(iter->second);
    //    }
    //    avg /= k;
    //    /* int sizeTemp;
    //    do {
    //        sizeTemp = pointsTemp.size();
    //        pointsTemp = filter(pointsTemp, threshold);
    //    } while (sizeTemp > pointsTemp.size());
    //    std::cout << pointsTemp.size() << std::endl;*/
    //    CGAL::edge_aware_upsample_point_set<CGAL::Parallel_if_available_tag>(pointsTemp, std::back_inserter(pointsTemp),
    //        CGAL::parameters::
    //        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
    //        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
    //        sharpness_angle(sharpnessAngle).
    //        edge_sensitivity(edgeSensitivity).
    //        neighbor_radius(avg * 5.0).
    //        number_of_output_points((int)(size * avg * avg)));
    //    std::vector<Vertex> verticesTemp = fromPointNormal(pointsTemp);
    //    vertices.insert(vertices.end(), verticesTemp.begin(), verticesTemp.end());
    //}

    std::vector<PointNormal> points = toPointNormal(this->vertices);
    CGAL::edge_aware_upsample_point_set<CGAL::Parallel_if_available_tag>(points, std::back_inserter(points),
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
        sharpness_angle(sharpnessAngle).
        edge_sensitivity(edgeSensitivity).
        neighbor_radius(neighborRadius).
        number_of_output_points(size));
    std::vector<Vertex> vertices = fromPointNormal(points);

    return PointSet(vertices);
}

PointSet PointSet::smooth(int k) {
    std::vector<Point> points = toPoint(vertices);
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, k);

    std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

Mesh PointSet::reconstruct(double maximumFacetLength) {
    std::vector<Point> points = toPoint(this->vertices);
    CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(points.begin(), points.end());
    reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(maximumFacetLength));

    std::vector<Vertex> vertices = this->vertices;
    std::vector<unsigned int> indices;
    for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
        for (unsigned int index : *iter)
            indices.push_back(index);
    std::cout << indices.size() / 3 << " facet(s) generated by reconstruction." << std::endl;

    return Mesh(vertices, indices);
}

void PointSet::render() {
    glPointSize(5);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);
}