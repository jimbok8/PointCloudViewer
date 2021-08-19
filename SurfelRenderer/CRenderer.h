#ifndef RENDERER_H
#define RENDERER_H

#include <Windows.h>
#include <vector>
#include <iostream>

#include "CSurfel.h"
#include "MyDataTypes.h"
#include "Matrix.h"
#include "Warper.h"
#include "ZBuffer.h"
#include "Shader.h"

class CRenderer {
private:
    int m_width, m_height;
    unsigned char* m_image;
    COLORREF m_backgroundColor;
    std::vector<CSurfel> m_surfels;
    MyDataTypes::CameraPosition m_cameraPosition;

    Warper* warper;
    ZBuffer* zBuffer;
    Shader* shader;

    void init();

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