#include "Shader.h"

Shader::Shader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath, const std::string& geometryShaderPath) {
    bool vertexShaderSuccess;
    int vertexShader = processShader(vertexShaderPath, GL_VERTEX_SHADER, vertexShaderSuccess);
    if (!vertexShaderSuccess) {
        this->program = 0;
        return;
    }

    bool fragmentShaderSuccess;
    int fragmentShader = processShader(fragmentShaderPath, GL_FRAGMENT_SHADER, fragmentShaderSuccess);
    if (!fragmentShaderSuccess) {
        this->program = 0;
        return;
    }

    bool geometryShaderSuccess;
    int geometryShader;
    if (geometryShaderPath != "") {
        geometryShader = processShader(geometryShaderPath, GL_GEOMETRY_SHADER, geometryShaderSuccess);
        if (!geometryShaderSuccess) {
            this->program = 0;
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
        this->program = 0;
        return;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    if (geometryShaderPath != "")
        glDeleteShader(geometryShader);

    this->program = program;
}

Shader::~Shader() {}

int Shader::processShader(const std::string& path, const unsigned int type, bool& success) const {
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

void Shader::use() const {
    glUseProgram(program);
}

void Shader::setVector3D(const std::string& name, const Vector3D& value) const {
    glUniform3fv(glGetUniformLocation(program, name.c_str()), 1, &value.values[0]);
}

void Shader::setMatrix4D(const std::string& name, const Matrix4D& value) const {
    glUniformMatrix4fv(glGetUniformLocation(program, name.c_str()), 1, GL_FALSE, &value.values[0][0]);
}