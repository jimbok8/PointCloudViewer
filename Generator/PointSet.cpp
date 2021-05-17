#include "PointSet.h"

PointSet::PointSet() {}

PointSet::PointSet(std::vector<Vertex> &vertices) {
    this->vertices = vertices;
    
    std::vector<PointNormal> points;
    for (const Vertex& vertex : this->vertices) {
        Point position(vertex.position.x, vertex.position.y, vertex.position.z);
        Vector normal(vertex.normal.x, vertex.normal.y, vertex.normal.z);
        points.push_back(std::make_pair(position, normal));
    }

    CGAL::pca_estimate_normals<CGAL::Sequential_tag>(points, 50, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    CGAL::mst_orient_normals(points, 50, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));

    for (int i = 0; i < this->vertices.size(); i++)
        this->vertices[i].normal = glm::vec3(points[i].second.x(), points[i].second.y(), points[i].second.z());

    unsigned int vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), this->vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

PointSet::~PointSet() {}

std::vector<Vertex> PointSet::getVertices() const {
    return vertices;
}

void PointSet::render() {
    glPointSize(5);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);
}