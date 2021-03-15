#ifndef K_D_TREE_H
#define K_D_TREE_H

#include <vector>

#include "Point.h"

class KDTree {
private:
    int index;
    float middle;
    std::vector<Point> points;
    KDTree *leftChild, *rightChild;

};

#endif