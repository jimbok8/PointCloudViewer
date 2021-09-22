#ifndef COMPUTE_SHADER_H
#define COMPUTE_SHADER_H

#include <cstring>

#include "CShader.h"

class CComputeShader : public CShader {
public:
    CComputeShader(const std::string& computeShaderPath);
    ~CComputeShader();
};

#endif