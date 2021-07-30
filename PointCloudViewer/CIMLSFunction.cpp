#include "CIMLSFunction.h"

CIMLSFunction::CIMLSFunction(const std::vector<CPoint>& points, const float radius, const float epsilon) :
    CFunction(),
    m_points(points),
    m_radius2(radius * radius),
    m_epsilon2(epsilon * epsilon) {
    m_pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            m_pointArray[i][j] = points[i].m_position(j);
    m_tree = new ANNkd_tree(m_pointArray, points.size(), 3);

    for (int i = 0; i < m_points.size(); i++)
        m_num.push_back(m_tree->annkFRSearch(m_pointArray[i], m_epsilon2, 0));
}

CIMLSFunction::~CIMLSFunction() {
    annDeallocPts(m_pointArray);
    delete m_tree;
}

float CIMLSFunction::f(const Eigen::Vector3f& x) const {
    ANNpoint point = annAllocPt(3);
    for (int i = 0; i < 3; i++)
        point[i] = x(i);
    int k = m_tree->annkFRSearch(point, m_radius2, 0);
    ANNidxArray indices = new ANNidx[k];
    ANNdistArray distances = new ANNdist[k];
    m_tree->annkFRSearch(point, m_radius2, k, indices, distances);

    std::vector<float> weights, dots;
    for (int i = 0; i < k; i++) {
        Eigen::Vector3f diff = x - m_points[indices[i]].m_position;
        weights.push_back(std::exp(-diff.squaredNorm() / m_epsilon2) / m_num[indices[i]]);
        dots.push_back(diff.dot(m_points[indices[i]].m_normal));
    }

    float sum1 = 0.0f, sum2 = 0.0f;
    for (int i = 0; i < weights.size(); i++) {
        sum1 += weights[i] * dots[i];
        sum2 += weights[i];
    }

    delete[] indices;
    delete[] distances;

    return sum1 / sum2;
}