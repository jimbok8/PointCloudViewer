#include "CMeshBoundary.h"

CMeshBoundary::CMeshBoundary(const int p0, const int p1, const int p2) {
    std::vector<int> indices;
    indices.push_back(p0);
    indices.push_back(p1);
    indices.push_back(p2);
    m_boundaries.push_back(CBoundary(indices));
}

CMeshBoundary::~CMeshBoundary() {}

std::vector<CBoundary> CMeshBoundary::getBoundaries() const {
    return m_boundaries;
}

int CMeshBoundary::contain(const int p) const {
    for (int i = 0; i < m_boundaries.size(); i++)
        if (m_boundaries[i].contain(p))
            return i;

    return -1;
}

void CMeshBoundary::neighbors(const int p, int& last, int& next) const {
    for (const CBoundary& boundary : m_boundaries)
        if (boundary.contain(p)) {
            boundary.neighbors(p, last, next);
            return;
        }
}

void CMeshBoundary::insert(const int last, const int next, const int p) {
    for (auto iter = m_boundaries.begin(); iter != m_boundaries.end(); iter++)
        if (iter->contain(last) && iter->contain(next)) {
            iter->insert(last, next, p);
            return;
        }
}

void CMeshBoundary::erase(const int p) {
    for (auto iter = m_boundaries.begin(); iter != m_boundaries.end(); iter++)
        if (iter->contain(p)) {
            iter->erase(p);
            if (iter->size() < 3)
                m_boundaries.erase(iter);
            return;
        }
}

void CMeshBoundary::split(const int p0, const int p1, const int p2, const int p3) {
    for (auto iter = m_boundaries.begin(); iter != m_boundaries.end(); iter++)
        if (iter->contain(p0)) {
            CBoundary boundary = iter->split(p0, p1, p2, p3);
            if (iter->size() < 3)
                m_boundaries.erase(iter);
            if (boundary.size() >= 3)
                m_boundaries.push_back(boundary);
            return;
        }
}