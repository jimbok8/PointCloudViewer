#ifndef VECTOR_3D_H
#define VECTOR_3D_H

#include <math.h>

#include <cuda_runtime.h>

class Vector3D {
private:
    float p[3];

public:
    __host__ __device__ Vector3D();
    __host__ __device__ Vector3D(float p0, float p1, float p2);
    __host__ __device__ ~Vector3D();
    __host__ __device__ float& operator[](int index);
    __host__ __device__ const float& operator[](int index) const;
    __host__ __device__ Vector3D& operator=(const Vector3D& P);
    __host__ __device__ Vector3D& operator+=(const Vector3D& P);
    __host__ __device__ Vector3D& operator-=(const Vector3D& P);
    __host__ __device__ Vector3D& operator*=(float s);
    __host__ __device__ Vector3D& operator/=(float s);
    __host__ __device__ Vector3D operator+(const Vector3D& P) const;
    __host__ __device__ Vector3D operator-(const Vector3D& P) const;
    __host__ __device__ Vector3D operator*(float s) const;
    __host__ __device__ Vector3D operator/(float s) const;
    __host__ __device__ float getSquaredLength() const;
    __host__ __device__ float getLength() const;
    __host__ __device__ float normalize();
    __host__ __device__ static Vector3D crossProduct(const Vector3D& a, const Vector3D& b);
    __host__ __device__ static float dotProduct(const Vector3D& a, const Vector3D& b);
};

#endif