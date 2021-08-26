#ifndef RENDERER_H
#define RENDERER_H

#include <cmath>
#include <vector>
#include <iostream>

#include <GLFW/glfw3.h>

#include "TypeHelper.h"
#include "MatrixHelper.h"

extern void project(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int bbox[4]);
extern void projectGPU(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int bbox[4]);

class CRenderer {
private:
    const float M_PI = std::acos(-1.0f);

    int m_numSurfels, m_width, m_height;
    unsigned char m_backgroundR, m_backgroundG, m_backgroundB;
    bool m_useGpu;
    unsigned char* m_image;
    CameraPosition m_cameraPosition;

    Warper* m_warper;
    ZBufferProperty* m_zBufferProperty, * m_zBufferPropertyGpu;
    ZBufferItem* m_zBuffer, * m_zBufferGpu, * m_clearData;
    float* m_filterLUT, * m_filterLUTGpu;
    Surfel* m_surfels, * m_surfelsGpu;

    void init();
    unsigned char* getPixelPtr(const int x, const int y);
    void setColor(const int x, const int y, const unsigned char r, const unsigned char g, const unsigned char b);

    void setFrustum(float fofv, float aspect, float nearplane, float farplane);
    void setTrafo(const float trafo[16]);
    void lightSamplePhongR(const float r, const float g, const float b, const float nx, const float ny, const float nz, const float vx, const float vy, const float vz, float& resultR, float& resultG, float& resultB);
    void shadeZBuffer(int magfactor, int bbox[4]);
   
public:
    CRenderer(const int numSurfels, const Surfel* surfels, const int width, const int height, const unsigned char backgroundR, const unsigned char backgroundG, const unsigned char backgroundB, const bool useGpu);
    ~CRenderer();
    int getWidth() const;
    int getHeight() const;
    const unsigned char* getImage() const;
    void scale(const float dScaleX, const float dScaleY, const float dScaleZ);
    void translate(const float dx, const float dy, const float dz);
    void rotate(const float dAngle, const float x, const float y, const float z);
    void render();
};

#endif