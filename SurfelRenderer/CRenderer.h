#ifndef RENDERER_H
#define RENDERER_H

#include <Windows.h>
#include <vector>

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
    MyDataTypes::TransformationMatrix16f m_sceneViewMatrix;

    Warper* warper;						// the warper
    ZBuffer* zBuffer;					// the z-buffer
    Shader* shader;						// the shader

    unsigned char* getPixelPtr(const int x, const int y) const;
    void init();

public:
    CRenderer(const int width, const int height, const COLORREF backgroundColor);
    ~CRenderer();
    const unsigned char* getImage() const;
    void render();
};

#endif