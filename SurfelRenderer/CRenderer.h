#ifndef RENDERER_H
#define RENDERER_H

#include <Windows.h>
#include <vector>
#include <iostream>

#include <GLFW/glfw3.h>

#include "CSurfel.h"
#include "MyDataTypes.h"
#include "Matrix.h"
#include "Warper.h"
#include "ZBuffer.h"

class CRenderer {
private:
    int m_width, m_height;
    unsigned char* m_image;
    COLORREF m_backgroundColor;
    std::vector<CSurfel> m_surfels;
    MyDataTypes::CameraPosition m_cameraPosition;

    Warper* warper;
    ZBuffer* zBuffer;

    float _shd_nx_c, _shd_ny_c, _shd_nz_c;
    float _shd_vx, _shd_vy, _shd_vz;
    MyDataTypes::RGBTriple _shd_Id;
    MyDataTypes::RGBTriple _shd_Ir;

    void init();
    unsigned char* getPixelPtr(const int x, const int y);
    void setColor(const int x, const int y, const COLORREF newPixelColor);

    void wrpSetFrustum(float fofv, float aspect, float nearplane, float farplane);
    void wrpSetTrafo(const float trafo[16]);
    void shdLightSamplePhong_R(float _shd_kA, float _shd_kD, float _shd_kS, unsigned char _shd_shininess, MyDataTypes::RGBTriple _shd_specularColor);
    void shdShadeZBuffer(int magfactor, int bbox[4]);

    static void unpackRGB(float& r, float& g, float& b, COLORREF color);
    static int zbfSurfaceSplat(int width, int height, ZBuffer* zBuffer, float x0, float y0, float z, float n[3], CSurfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int bbox[4]);
    static void wrpProjectSample(int width, int height, Warper* warper, ZBuffer* zBuffer, CSurfel* surfel, int bbox[4]);

public:
    CRenderer(const std::vector<CSurfel>& surfels, const int width, const int height, const COLORREF backgroundColor);
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