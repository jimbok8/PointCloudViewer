#ifndef SHADER_H
#define SHADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <glad/glad.h>

class Shader {
private:
    unsigned int program;
    int processShader(const std::string& path, const unsigned int type, bool& success) const;

public:
    Shader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath, const std::string& geometryShaderPath = "");
    ~Shader();
    void use() const;
    void setVector3D(const std::string& name, const Eigen::Vector3f& value) const;
    void setMatrix4D(const std::string& name, const Eigen::Matrix4f& value) const;
};

#endif