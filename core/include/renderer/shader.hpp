#pragma once
#include "types.hpp"

#include <string>

class Shader {
public:
    int success = 0;
    char infoLog[512] = "";

    Shader(const char* vertexSource, const char* fragSource);
    ~Shader();

    operator int() const;

    void setUniformf(const char* name, float value) const;
    void setUniform4f(const char* name, float values[]) const;
    void setUniformM4f(const char* name, const float* matrix) const;

private:
    unsigned int vs = 0, fs = 0, program = 0;

    std::string loadShaderSource(const char* path);
    unsigned int compileShader(const char* source, DataType type);
    unsigned int linkShaders() const;
};