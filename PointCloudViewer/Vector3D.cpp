#include "Vector3D.h"

Vector3D::Vector3D(const float x, const float y, const float z) :
    values{x, y, z} {}

Vector3D::~Vector3D() {}

Vector3D Vector3D::operator +(const Vector3D& v) const {
    return Vector3D(values[0] + v.values[0], values[1] + v.values[1], values[2] + v.values[2]);
}

Vector3D Vector3D::operator -(const Vector3D& v) const {
    return Vector3D(values[0] - v.values[0], values[1] - v.values[1], values[2] - v.values[2]);
}

Vector3D Vector3D::operator *(const float x) const {
    return Vector3D(values[0] * x, values[1] * x, values[2] * x);
}

Vector3D Vector3D::operator /(const float x) const {
    return Vector3D(values[0] / x, values[1] / x, values[2] / x);
}

float Vector3D::length() const {
    return std::sqrt(dot(*this));
}

Vector3D Vector3D::normalize() const {
    return *this / length();
}

float Vector3D::dot(const Vector3D& v) const {
    float sum = 0.0f;
    for (int i = 0; i < 3; i++)
        sum += values[i] * v.values[i];
    return sum;
}

Vector3D Vector3D::cross(const Vector3D& v) const {
    return Vector3D(values[1] * v.values[2] - values[2] - v.values[1],
        values[2] * v.values[0] - values[0] * v.values[2],
        values[0] * v.values[1] - values[1] * v.values[0]);
}