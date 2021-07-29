#include "CIMLSFunction.h"

CIMLSFunction::CIMLSFunction(const std::vector<CPoint>& points, const float epsilon) :
    CFunction(),
    m_points(points),
    m_epsilon2(epsilon * epsilon) {
    ANNpointArray pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].m_position(j);
    ANNkd_tree* tree = new ANNkd_tree(pointArray, points.size(), 3);

    for (int i = 0; i < m_points.size(); i++)
        m_num.push_back(tree->annkFRSearch(pointArray[i], m_epsilon2, 0));

    annDeallocPts(pointArray);
    delete tree;
}

CIMLSFunction::~CIMLSFunction() {}

float CIMLSFunction::f(const Eigen::Vector3f& x) const {
    std::vector<float> weights, dots;
    for (int i = 0; i < m_points.size(); i++) {
        Eigen::Vector3f diff = x - m_points[i].m_position;
        weights.push_back(std::exp(-diff.squaredNorm() / m_epsilon2) / m_num[i]);
        dots.push_back(diff.dot(m_points[i].m_normal));
    }

    float sum1 = 0.0f, sum2 = 0.0f;
    for (int i = 0; i < m_points.size(); i++) {
        sum1 += weights[i] * dots[i];
        sum2 += weights[i];
    }

    return sum1 / sum2;
}