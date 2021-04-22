#include "CMesh.h"

CMesh::CMesh(const std::vector<CPoint>& points, const std::vector<unsigned int>& indices) :
    m_points(points),
    m_indices(indices) {
    //calculateNormals();

    unsigned vbo, ebo;
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, m_points.size() * sizeof(CPoint), m_points.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_normal));

    glBindVertexArray(0);
}

CMesh::~CMesh() {}

void CMesh::calculateNormals() {
    std::vector<std::vector<Eigen::Vector3f>> normals(m_points.size());
    for (auto iter = m_indices.begin(); iter != m_indices.end(); ) {
        int x = *(iter++);
        int y = *(iter++);
        int z = *(iter++);
        Eigen::Vector3f a = m_points[y].m_position - m_points[x].m_position;
        Eigen::Vector3f b = m_points[z].m_position - m_points[y].m_position;
        Eigen::Vector3f normal = a.cross(b).normalized();
        normals[x].push_back(normal);
        normals[y].push_back(normal);
        normals[z].push_back(normal);
    }

    for (int i = 0; i < m_points.size(); i++)
        if (!normals[i].empty())
            m_points[i].m_normal = (std::accumulate(normals[i].begin(), normals[i].end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f)) / (float)normals[i].size()).normalized();
}

void CMesh::render() const {
    glBindVertexArray(m_vao);
    glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}