#include "Mesh.h"

Mesh::Mesh(const std::vector<Point>& points, const std::vector<unsigned int>& indices) :
points(points),
indices(indices) {
    calculateNormals();

    unsigned vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, this->points.size() * sizeof(Point), this->points.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(unsigned int), this->indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, normal));

    glBindVertexArray(0);
}

Mesh::~Mesh() {}

void Mesh::calculateNormals() {
    std::vector<std::vector<Eigen::Vector3f>> normals(points.size());
    for (auto iter = indices.begin(); iter != indices.end(); ) {
        int x = *(iter++);
        int y = *(iter++);
        int z = *(iter++);
        Eigen::Vector3f a = points[y].position - points[x].position;
        Eigen::Vector3f b = points[z].position - points[y].position;
        Eigen::Vector3f normal = a.cross(b).normalized();
        normals[x].push_back(normal);
        normals[y].push_back(normal);
        normals[z].push_back(normal);
    }

    for (int i = 0; i < points.size(); i++)
        if (!normals[i].empty())
            points[i].normal = (std::accumulate(normals[i].begin(), normals[i].end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f)) / (float)normals[i].size()).normalized();
}

void Mesh::render() const {
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}