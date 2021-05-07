#ifndef BOUNDARY_H
#define BOUNDARY_H

#include <vector>
#include <iostream>

class CBoundary {
private:
    std::vector<int> m_indices;

public:
    CBoundary(const std::vector<int>& indices);
    ~CBoundary();
    int size() const;
    bool contain(const int p) const;
    void neighbors(const int p, int& last, int& next) const;
    void insert(const int last, const int next, const int p);
    void erase(const int p);
    CBoundary split(const int p0, const int p1, const int p2, const int p4);
};

#endif