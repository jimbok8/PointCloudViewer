#ifndef VECTOR_3D_H
#define VECTOR_3D_H

#include <cmath>

class Vector3D {
public:
    float values[3];
    Vector3D(const float x, const float y, const float z);
    ~Vector3D();
    Vector3D operator +(const Vector3D& v) const;
    Vector3D operator -(const Vector3D& v) const;
    Vector3D operator *(const float x) const;
    Vector3D operator /(const float x) const;
    float length() const;
    Vector3D normalize() const;
    float dot(const Vector3D& v) const;
    Vector3D cross(const Vector3D& v) const;
};

#endif