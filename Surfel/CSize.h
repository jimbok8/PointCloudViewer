#ifndef SIZE_H
#define SIZE_H

class CSize {
public:
    int cx, cy;
    CSize();
    CSize(const int x, const int y);
    bool operator == (const CSize& size) const;
};

#endif