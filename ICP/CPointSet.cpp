#include "CPointSet.h"

CPointSet::CPointSet(const std::vector<CPoint>& points0, const std::vector<CPoint>& points1, const std::vector<int>& indices) {
    for (const CPoint& point: points0) {
        m_points0.push_back(CPoint(point.m_position, point.m_normal, Eigen::Vector3f(1.0f, 0.0f, 0.0f)));
        m_points1.push_back(CPoint(point.m_position, point.m_normal, Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
    }
    for (const CPoint& point : points1) {
        m_points0.push_back(CPoint(point.m_position, point.m_normal, Eigen::Vector3f(0.0f, 1.0f, 0.0f)));
        m_points1.push_back(CPoint(point.m_position, point.m_normal, Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
    }
    for (unsigned int i = 0; i < indices.size(); i++)
        if (indices[i] > -1) {
            m_indices.push_back(i);
            m_indices.push_back(points0.size() + indices[i]);
        }

    unsigned int vbo0;
    glGenVertexArrays(1, &m_vao0);
    glGenBuffers(1, &vbo0);

    glBindVertexArray(m_vao0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo0);
    glBufferData(GL_ARRAY_BUFFER, m_points0.size() * sizeof(CPoint), m_points0.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_color));

    glBindVertexArray(0);

    unsigned int vbo1, ebo;
    glGenVertexArrays(1, &m_vao1);
    glGenBuffers(1, &vbo1);
    glGenBuffers(1, &ebo);

    glBindVertexArray(m_vao1);
    glBindBuffer(GL_ARRAY_BUFFER, vbo1);
    glBufferData(GL_ARRAY_BUFFER, m_points1.size() * sizeof(CPoint), m_points1.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_color));

    glBindVertexArray(0);
}

CPointSet::~CPointSet() {}

void CPointSet::render(const bool flag) const {
    glPointSize(5);
    glBindVertexArray(m_vao0);
    glDrawArrays(GL_POINTS, 0, m_points0.size());
    glBindVertexArray(0);

    if (flag) {
        glBindVertexArray(m_vao1);
        glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}