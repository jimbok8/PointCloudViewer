#include "CMesh.h"

CMesh::CMesh(const std::vector<CPoint>& points, const std::vector<unsigned int>& indices) {
    ANNpointArray pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].m_position(j);
    ANNkd_tree* tree = new ANNkd_tree(pointArray, points.size(), 3);

    unsigned int num = 0;
    std::vector<unsigned int> mapping(points.size(), UINT_MAX);
    for (int i = 0; i < points.size(); i++)
        if (mapping[i] == UINT_MAX) {
            int k = tree->annkFRSearch(pointArray[i], 1e-6f, 0);
            ANNidxArray indices = new ANNidx[k];
            ANNdistArray distances = new ANNdist[k];
            tree->annkFRSearch(pointArray[i], 1e-6f, k, indices, distances);
            
            for (int j = 0; j < k; j++)
                mapping[indices[j]] = num;
            num++;
            m_points.push_back(points[i]);

            delete[] indices;
            delete[] distances;
        }
    for (unsigned int index : indices)
        m_indices.push_back(mapping[index]);

    smooth();

    calculateNormals();

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

    annDeallocPts(pointArray);
    delete tree;
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

void CMesh::smooth() {
    std::vector<std::vector<unsigned int>> neighbors(m_points.size());
    for (int j = 0; j < m_indices.size(); j += 3) {
        int x = m_indices[j];
        int y = m_indices[j + 1];
        int z = m_indices[j + 2];
        neighbors[x].push_back(y);
        neighbors[x].push_back(z);
        neighbors[y].push_back(x);
        neighbors[y].push_back(z);
        neighbors[z].push_back(x);
        neighbors[z].push_back(y);
    }

    for (int iter = 0; iter < 3; iter++) {
        std::vector<CPoint> newPoints;
        for (int i = 0; i < m_points.size(); i++) {
            Eigen::Vector3f sum = m_points[i].m_position;
            for (const unsigned int neighbor : neighbors[i])
                sum += m_points[neighbor].m_position;
            newPoints.push_back(CPoint(sum / (neighbors[i].size() + 1)));
        }
        m_points = newPoints;
    }
}

std::vector<unsigned int> CMesh::getIndices() const {
    return m_indices;
}

void CMesh::render() const {
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}