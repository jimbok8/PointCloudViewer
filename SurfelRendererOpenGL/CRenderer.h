#ifndef RENDERER_H
#define RENDERER_H

#include <cmath>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "TypeHelper.h"
#include "MatrixHelper.h"
#include "TransformHelper.h"
#include "CComputeShader.h"
#include "CRenderShader.h"

class CRenderer {
private:
    const float M_PI = std::acos(-1.0f);

    int m_numSurfels, m_width, m_height;
    unsigned char m_backgroundR, m_backgroundG, m_backgroundB;
    unsigned char* m_image;
    CameraPosition m_cameraPosition;

    Warper* m_warper;
    ZBufferProperty* m_zBufferProperty;
    ZBufferItem* m_zBuffer, * m_clearData;
    float* m_filterLUT, * m_floatImage;
    Surfel* m_surfels;
    Vertex* m_vertices;
    unsigned int* m_indices;

    unsigned int m_vao, m_ssbo0, m_ssbo1, m_ssbo2, m_ssbo3, m_ssbo4, m_ssbo5, m_ssbo6;

    void init();
    void setFrustum(float fofv, float aspect, float nearplane, float farplane);
    void setTrafo(const float trafo[16]);
    void project();
    void splat();
    void shade();

public:
    CRenderer(const int numSurfels, const Surfel* surfels, const int width, const int height, const unsigned char backgroundR, const unsigned char backgroundG, const unsigned char backgroundB);
    ~CRenderer();
    int getWidth() const;
    int getHeight() const;
    const unsigned char* getImage() const;
    void resize(const int width, const int height);
    void scale(const float dScaleX, const float dScaleY, const float dScaleZ);
    void translate(const float dx, const float dy, const float dz);
    void rotate(const float dAngle, const float x, const float y, const float z);
    void render();
};

#endif