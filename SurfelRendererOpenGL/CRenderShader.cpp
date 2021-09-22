#include "CRenderShader.h"

CRenderShader::CRenderShader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath, const std::string& geometryShaderPath) :
    CShader() {
    bool vertexShaderSuccess;
    int vertexShader = processShader(vertexShaderPath, GL_VERTEX_SHADER, vertexShaderSuccess);
    if (!vertexShaderSuccess) {
        m_program = 0;
        return;
    }

    bool fragmentShaderSuccess;
    int fragmentShader = processShader(fragmentShaderPath, GL_FRAGMENT_SHADER, fragmentShaderSuccess);
    if (!fragmentShaderSuccess) {
        m_program = 0;
        return;
    }

    bool geometryShaderSuccess;
    int geometryShader;
    if (geometryShaderPath != "") {
        geometryShader = processShader(geometryShaderPath, GL_GEOMETRY_SHADER, geometryShaderSuccess);
        if (!geometryShaderSuccess) {
            m_program = 0;
            return;
        }
    }

    int program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    if (geometryShaderPath != "")
        glAttachShader(program, geometryShader);
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
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    if (geometryShaderPath != "")
        glDeleteShader(geometryShader);

    m_program = program;
}

CRenderShader::~CRenderShader() {}