#ifndef MESH_BOUNDARY_H
#define MESH_BOUNDARY_H

#include <vector>

#include "CBoundary.h"

class CMeshBoundary {
private:
    std::vector<CBoundary> m_boundaries;

public:
    CMeshBoundary(const int p0, const int p1, const int p2);
    ~CMeshBoundary();
    std::vector<CBoundary> getBoundaries() const;
    int contain(const int p) const;
    void neighbors(const int p, int& last, int& next) const;
    void insert(const int last, const int next, const int p);
    void erase(const int p);
    void split(const int p0, const int p1, const int p2, const int p3);
};

#endif