#include "CComputeShader.h"

CComputeShader::CComputeShader(const std::string& computeShaderPath) :
    CShader() {
    bool computeShaderSuccess;
    int computeShader = processShader(computeShaderPath, GL_COMPUTE_SHADER, computeShaderSuccess);
    if (!computeShaderSuccess) {
        m_program = 0;
        return;
    }

    int program = glCreateProgram();
    glAttachShader(program, computeShader);
    glLinkProgram(program);
    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info[512];
        glGetProgramInfoLog(program, 512, nullptr, info);
        std::cerr << "Failed to link shader program:" << std::endl << info << std::endl;
        m_program = 0;
        return;
    }
    glDeleteShader(computeShader);

    m_program = program;
}

CComputeShader::~CComputeShader() {}