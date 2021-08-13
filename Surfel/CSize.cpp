#include "CSize.h"

CSize::CSize() :
    cx(0),
    cy(0) {}

CSize::CSize(const int x, const int y) :
    cx(x),
    cy(y) {}

bool CSize::operator == (const CSize& size) const {
    return cx == size.cx && cy == size.cy;
}