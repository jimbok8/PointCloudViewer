#include "CPointSet.h"

CPointSet::CPointSet(const std::vector<CPoint>& points0, const std::vector<CPoint>& points1, const std::vector<int>& indices) {
    for (const CPoint& point : points0)
        m_points.push_back(point);
    for (const CPoint& point : points1)
        m_points.push_back(point);
    for (unsigned int i = 0; i < indices.size(); i++)
        if (indices[i] > -1) {
            m_indices.push_back(i);
            m_indices.push_back(points0.size() + indices[i]);
        }

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

CPointSet::~CPointSet() {}

void CPointSet::render() const {
    glPointSize(5);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_points.size());
    glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}