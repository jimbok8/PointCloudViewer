#ifndef RENDER_SHADER_H
#define RENDER_SHADER_H

#include <cstring>

#include "CShader.h"

class CRenderShader : public CShader {
public:
    CRenderShader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath, const std::string& geometryShaderPath = "");
    ~CRenderShader();
};

#endif