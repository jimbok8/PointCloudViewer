#ifndef MATRIX_4D_H
#define MATRIX_4D_H

#include <cstring>

#include "Vector3D.h"

class Matrix4D {
public:
    float values[4][4];
    Matrix4D();
    Matrix4D(const float x);
    ~Matrix4D();
    Matrix4D operator *(const float x) const;
    Matrix4D operator *(const Matrix4D& m) const;
    static Matrix4D rotate(const Vector3D& v, const float angle);
    static Matrix4D lookAt(const Vector3D& camera, const Vector3D& center, const Vector3D& up);
    static Matrix4D perspective(const float fovy, const float aspect, const float zNear, const float zFar);
};

#endif