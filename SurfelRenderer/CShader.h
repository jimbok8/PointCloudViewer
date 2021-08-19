#ifndef CSHADER_H
#define CSHADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <glad/glad.h>

class CShader {
private:
    unsigned int m_program;
    int processShader(const std::string& path, const unsigned int type, bool& success) const;

public:
    CShader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath, const std::string& geometryShaderPath = "");
    ~CShader();
    void use() const;
    void setVector3D(const std::string& name, const Eigen::Vector3f& value) const;
    void setMatrix4D(const std::string& name, const Eigen::Matrix4f& value) const;
    void setInt(const std::string& name, const int value) const;
};

#endif