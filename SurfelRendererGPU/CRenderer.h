#ifndef RENDERER_H
#define RENDERER_H

#include <cmath>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

#include "TypeHelper.h"
#include "MatrixHelper.h"
#include "TransformHelper.h"

extern void project(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, float factor, Eigen::Matrix4f& translate, Eigen::Matrix4f& rotate);
extern void projectGpu(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels);
extern void shade(int width, int height, Warper* warper, ZBufferItem* zBuffer, unsigned char* image, unsigned char backgroundR, unsigned char backgroundG, unsigned char backgroundB);
extern void shadeGpu(int width, int height, Warper* warper, ZBufferItem* zBuffer, unsigned char* image, unsigned char backgroundR, unsigned char backgroundG, unsigned char backgroundB);

class CRenderer {
private:
    const float M_PI = std::acos(-1.0f);

    int m_numSurfels, m_width, m_height;
    unsigned char m_backgroundR, m_backgroundG, m_backgroundB;
    bool m_useGpu;
    unsigned char* m_image, * m_imageGpu;
    CameraPosition m_cameraPosition;

    Warper* m_warper, * m_warperGpu;
    ZBufferProperty* m_zBufferProperty, * m_zBufferPropertyGpu;
    ZBufferItem* m_zBuffer, * m_zBufferGpu, * m_clearData, * m_clearDataGpu;
    float* m_filterLUT, * m_filterLUTGpu;
    Surfel* m_surfels, * m_surfelsGpu;

    float m_factor;
    Eigen::Matrix4f m_translate, m_rotate;

    void init();
    void setFrustum(float fofv, float aspect, float nearplane, float farplane);
    void setTrafo(const float trafo[16]);
   
public:
    CRenderer(const int numSurfels, const Surfel* surfels, const int width, const int height, const unsigned char backgroundR, const unsigned char backgroundG, const unsigned char backgroundB, const bool useGpu);
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