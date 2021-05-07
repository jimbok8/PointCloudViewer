#include "CBoundary.h"

CBoundary::CBoundary(const std::vector<int>& indices) :
    m_indices(indices) {}

CBoundary::~CBoundary() {}

int CBoundary::size() const {
    return m_indices.size();
}

bool CBoundary::contain(const int p) const {
    for (const int index : m_indices)
        if (index == p)
            return true;

    return false;
}

void CBoundary::neighbors(const int p, int& last, int& next) const {
    for (int i = 0; i < m_indices.size(); i++)
        if (m_indices[i] == p) {
            last = (i == 0 ? *m_indices.rbegin() : m_indices[i - 1]);
            next = (i == m_indices.size() - 1 ? *m_indices.begin() : m_indices[i + 1]);
            return;
        }
}

void CBoundary::insert(const int last, const int next, const int p) {
    for (auto iter = m_indices.begin(); iter != m_indices.end(); iter++)
        if (*iter == next) {
            m_indices.insert(iter, p);
            return;
        }
}

void CBoundary::erase(const int p) {
    for (auto iter = m_indices.begin(); iter != m_indices.end(); iter++)
        if (*iter == p) {
            m_indices.erase(iter);
            return;
        }
}

CBoundary CBoundary::split(const int p0, const int p1, const int p2, const int p3) {
    std::cout << m_indices.size() << std::endl;
    for (auto index : m_indices)
        std::cout << index << ' ';
    std::cout << std::endl << p0 << ' ' << p1 << ' ' << p2 << ' ' << p3 << std::endl;

    auto iter = m_indices.begin();
    for (; iter != m_indices.end(); iter++)
        if (*iter == p0)
            break;
    
    std::vector<int> indices;
    iter++;
    if (iter == m_indices.end())
        iter = m_indices.begin();
    while (*iter != p3) {
        indices.push_back(*iter);
        iter = m_indices.erase(iter);
        if (iter == m_indices.end())
            iter = m_indices.begin();
    }
    
    return CBoundary(indices);
}