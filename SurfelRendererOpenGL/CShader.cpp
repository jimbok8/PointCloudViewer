#include "CShader.h"

CShader::CShader() {}

CShader::~CShader() {}

int CShader::processShader(const std::string& path, const unsigned int type, bool& success) const {
    std::ifstream fileStream(path);
    if (!(success = fileStream.is_open())) {
        std::cerr << "Failed to open shader file " << path << std::endl;
        return 0;
    }

    std::stringstream stringStream;
    stringStream << fileStream.rdbuf();
    fileStream.close();
    std::string source = stringStream.str();

    int shader = glCreateShader(type);
    const char* pointer = source.c_str();
    glShaderSource(shader, 1, &pointer, nullptr);
    glCompileShader(shader);
    int compileSuccess;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compileSuccess);
    if (!(success = compileSuccess)) {
        char info[512];
        glGetShaderInfoLog(shader, 512, nullptr, info);
        std::cerr << "Failed to compile shader file " << path << ":" << std::endl << info << std::endl;
        return 0;
    }

    return shader;
}

void CShader::use() const {
    glUseProgram(m_program);
}

void CShader::setInt(const std::string& name, const int value) const {
    glUniform1i(glGetUniformLocation(m_program, name.c_str()), value);
}