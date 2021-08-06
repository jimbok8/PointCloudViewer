#include "CPointSet.h"

CPointSet::CPointSet(const std::string& path) {
    std::fstream::sync_with_stdio(false);
    std::ifstream fin(path);
    std::string s;

    m_minX = m_minY = m_minZ = FLT_MAX;
    m_maxX = m_maxY = m_maxZ = -FLT_MAX;

    float x, y, z, ux, uy, uz, vx, vy, vz;
    while (fin >> x >> y >> z >> ux >> uy >> uz >> vx >> vy >> vz) {
        Eigen::Vector3f position(x, y, z), u(ux, uy, uz), v(vx, vy, vz);

        m_minX = std::min(m_minX, x);
        m_maxX = std::max(m_maxX, x);
        m_minY = std::min(m_minY, y);
        m_maxY = std::max(m_maxY, y);
        m_minZ = std::min(m_minZ, z);
        m_maxZ = std::max(m_maxZ, z);

        float radius = u.norm() * 0.5f;
        u *= 0.5f;
        v *= 0.5f;
        //u = u.normalized() * radius;
        //v = v.normalized() * radius;
        Eigen::Vector3f normal = u.cross(v);

        m_points.push_back(CPoint(position, normal));
        unsigned int i0 = m_vertices.size();
        m_vertices.push_back(CPoint(position + u + v, normal, position, radius));
        unsigned int i1 = m_vertices.size();
        m_vertices.push_back(CPoint(position - u + v, normal, position, radius));
        unsigned int i2 = m_vertices.size();
        m_vertices.push_back(CPoint(position - u - v, normal, position, radius));
        unsigned int i3 = m_vertices.size();
        m_vertices.push_back(CPoint(position + u - v, normal, position, radius));
        m_indices.push_back(i0);
        m_indices.push_back(i1);
        m_indices.push_back(i2);
        m_indices.push_back(i0);
        m_indices.push_back(i2);
        m_indices.push_back(i3);

        std::getline(fin, s);
    }

    Eigen::Vector3f center((m_minX + m_maxX) * 0.5f, (m_minY + m_maxY) * 0.5f, (m_minZ + m_maxZ) * 0.5f);
    for (CPoint& point : m_points)
        point.m_position -= center;
    for (CPoint& point : m_vertices) {
        point.m_position -= center;
        point.m_center -= center;
    }

    unsigned vbo, ebo;
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(CPoint), m_vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_center));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_radius));

    glBindVertexArray(0);
}

CPointSet::~CPointSet() {}

float CPointSet::getMinX() const {
    return m_minX;
}

float CPointSet::getMaxX() const {
    return m_maxX;
}

float CPointSet::scale() const {
    return std::max(m_maxX - m_minX, std::max(m_maxY - m_minY, m_maxZ - m_minZ));
}

void CPointSet::render() const {
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}