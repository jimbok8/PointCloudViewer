#include "Matrix4D.h"

Matrix4D::Matrix4D() :
    values{{0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f}} {}

Matrix4D::Matrix4D(const float x) :
    values{{x, 0.0f, 0.0f, 0.0f},
        {0.0f, x, 0.0f, 0.0f},
        {0.0f, 0.0f, x, 0.0f},
        {0.0f, 0.0f, 0.0f, x}} {}

Matrix4D::~Matrix4D() {}

Matrix4D Matrix4D::operator *(const float x) const {
    Matrix4D ans;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ans.values[i][j] = values[i][j] * x;
    return ans;
}

Matrix4D Matrix4D::operator *(const Matrix4D& m) const {
    Matrix4D ans;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            ans.values[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                ans.values[i][j] += values[i][k] * m.values[k][j];
        }
    return ans;
}

Matrix4D Matrix4D::rotate(const Vector3D& v, const float angle) {
    Vector3D axis = v.normalize();
    float s = std::sin(angle);
    float c = std::cos(angle);

    Matrix4D ans;
    ans.values[0][0] = (1.0f - c) * axis.values[0] * axis.values[0] + c;
    ans.values[0][1] = (1.0f - c) * axis.values[0] * axis.values[1] - s * axis.values[2];
    ans.values[0][2] = (1.0f - c) * axis.values[0] * axis.values[2] + s * axis.values[1];

    ans.values[1][0] = (1.0f - c) * axis.values[1] * axis.values[0] + s * axis.values[2];
    ans.values[1][1] = (1.0f - c) * axis.values[1] * axis.values[1] + c;
    ans.values[1][2] = (1.0f - c) * axis.values[1] * axis.values[2] - s * axis.values[0];

    ans.values[2][0] = (1.0f - c) * axis.values[2] * axis.values[0] - s * axis.values[1];
    ans.values[2][1] = (1.0f - c) * axis.values[2] * axis.values[1] + s * axis.values[0];
    ans.values[2][2] = (1.0f - c) * axis.values[2] * axis.values[2] + c;

    ans.values[3][3] = 1;

    return ans;
}

Matrix4D Matrix4D::lookAt(const Vector3D& camera, const Vector3D& center, const Vector3D& up) {
    Vector3D f = (center - camera).normalize();
    Vector3D s = f.cross(up).normalize();
    Vector3D u = s.cross(f);

    Matrix4D ans;
    ans.values[0][0] = s.values[0];
    ans.values[0][1] = u.values[0];
    ans.values[0][2] = -f.values[0];

    ans.values[1][0] = s.values[1];
    ans.values[1][1] = u.values[1];
    ans.values[1][2] = -f.values[1];

    ans.values[2][0] = s.values[2];
    ans.values[2][1] = u.values[2];
    ans.values[2][2] = -f.values[2];

    ans.values[3][0] = -s.dot(camera);
    ans.values[3][1] = -u.dot(camera);
    ans.values[3][2] = f.dot(camera);

    return ans;
}

Matrix4D Matrix4D::perspective(const float fovy, const float aspect, const float zNear, const float zFar) {
    float t = std::tan(fovy * 0.5f);

    Matrix4D ans;
    ans.values[0][0] = 1.0f / (aspect * t);
    ans.values[1][1] = 1.0f / t;
    ans.values[2][2] = -(zNear + zFar) / (zFar - zNear);
    ans.values[2][3] = -1.0f;
    ans.values[3][2] = -2.0f * zNear * zFar / (zFar - zNear);

    return ans;
}