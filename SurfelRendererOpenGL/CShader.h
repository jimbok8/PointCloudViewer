#ifndef SHADER_H
#define SHADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <glad/glad.h>

class CShader {
protected:
    unsigned int m_program;
    int processShader(const std::string& path, const unsigned int type, bool& success) const;

public:
    CShader();
    ~CShader();
    void use() const;
    void setInt(const std::string& name, const int value) const;
};

#endif