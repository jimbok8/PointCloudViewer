#include "Vector3D.h"

__host__ __device__ Vector3D::Vector3D() {}

__host__ __device__ Vector3D::Vector3D(float p0, float p1, float p2) {
	p[0] = p0;
	p[1] = p1;
	p[2] = p2;
}

__host__ __device__ Vector3D::~Vector3D() {}

__host__ __device__ float& Vector3D::operator[](int index) {
	return p[index];
}

__host__ __device__ const float& Vector3D::operator[](int index) const {
	return p[index];
}

__host__ __device__ Vector3D& Vector3D::operator=(const Vector3D& P) {
	p[0] = P[0];
	p[1] = P[1];
	p[2] = P[2];
	return (*this);
}

__host__ __device__ Vector3D& Vector3D::operator+=(const Vector3D& P) {
	p[0] += P[0];
	p[1] += P[1];
	p[2] += P[2];
	return (*this);
}

__host__ __device__ Vector3D& Vector3D::operator-=(const Vector3D& P) {
	p[0] -= P[0];
	p[1] -= P[1];
	p[2] -= P[2];
	return (*this);
}

__host__ __device__ Vector3D& Vector3D::operator*=(float s) {
	p[0] *= s;
	p[1] *= s;
	p[2] *= s;
	return (*this);
}

__host__ __device__ Vector3D& Vector3D::operator/=(float s) {
	p[0] /= s;
	p[1] /= s;
	p[2] /= s;
	return (*this);
}

__host__ __device__ Vector3D Vector3D::operator+(const Vector3D& P) const {
	Vector3D res;
	res[0] = p[0] + P[0];
	res[1] = p[1] + P[1];
	res[2] = p[2] + P[2];
	return (res);
}

__host__ __device__ Vector3D Vector3D::operator-(const Vector3D& P) const {
	Vector3D res;
	res[0] = p[0] - P[0];
	res[1] = p[1] - P[1];
	res[2] = p[2] - P[2];
	return (res);
}

__host__ __device__ Vector3D Vector3D::operator*(float s) const {
	Vector3D res;
	res[0] = p[0] * s;
	res[1] = p[1] * s;
	res[2] = p[2] * s;
	return (res);
}

__host__ __device__ Vector3D Vector3D::operator/(float s) const {
	Vector3D res;
	float invS = 1.0f / s;
	res[0] = p[0] * invS;
	res[1] = p[1] * invS;
	res[2] = p[2] * invS;
	return (res);
}

__host__ __device__ float Vector3D::getSquaredLength() const {
	return(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

__host__ __device__ float Vector3D::getLength() const {
	return (float)sqrt(getSquaredLength());
}

__host__ __device__ float Vector3D::normalize() {
	float length = getLength();
	if (length == 0.0f)
		return 0;

	float rezLength = 1.0f / length;
	p[0] *= rezLength;
	p[1] *= rezLength;
	p[2] *= rezLength;
	return length;
}

__host__ __device__ Vector3D Vector3D::crossProduct(const Vector3D& a, const Vector3D& b) {
	Vector3D result;

	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];

	return(result);
}


__host__ __device__ float Vector3D::dotProduct(const Vector3D& a, const Vector3D& b) {
	return(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}