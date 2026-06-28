#include "renderer/shader.hpp"

#include <fstream>
#include <string>

#define GLFW_INCLUDE_NONE
#include <app.hpp>
#include <format>
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <iterator>

Shader::Shader(const char* vertexSourceFile, const char* fragSourceFile) {
    std::string vertexSource = loadShaderSource(vertexSourceFile);
    if(!success) {
        return;
    }

    vs = compileShader(vertexSource.c_str(), GL_VERTEX_SHADER);
    glGetShaderiv(vs, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(vs, 512, nullptr, infoLog);
        App::log.error(std::format("Vertex Shader Compilation Error: {}\n", infoLog));
        return;
    }

    std::string fragSource = loadShaderSource(fragSourceFile);
    if(!success) {
        return;
    }

    fs = compileShader(fragSource.c_str(), GL_FRAGMENT_SHADER);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(fs, 512, nullptr, infoLog);
        App::log.error(std::format("Fragment Shader Compilation Error: {}\n", infoLog));
        return;
    }

    program = linkShaders();
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        App::log.error(std::format("Shader Linker Error: {}\n", infoLog));
        return;
    }
}

Shader::~Shader() {
    if(program)
        glDeleteProgram(program);
}

Shader::operator int() const {
    return program;
}

std::string Shader::loadShaderSource(const char* path) {
    std::ifstream source(path);

    if(!source.is_open()) {
        App::log.error(std::format("Failed to open shader: {}\n", infoLog));
        success = false;
        return "";
    } else {
        success = true;
    }

    return std::string(
        (std::istreambuf_iterator<char>(source)),
        std::istreambuf_iterator<char>()
    );
}

unsigned int Shader::compileShader(const char* source, GLenum type) {
    unsigned int shader;
    shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    return shader;
}

unsigned int Shader::linkShaders() const {
    unsigned int shaders;
    shaders = glCreateProgram();

    glAttachShader(shaders, vs);
    glAttachShader(shaders, fs);
    glLinkProgram(shaders);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return shaders;
}

void Shader::setUniformf(const char* name, float value) const {
    glUseProgram(program);
    glUniform1f(glGetUniformLocation(program, name), value);
    glUseProgram(0);
}

void Shader::setUniform4f(const char* name, float values[]) const {
    glUseProgram(program);
    glUniform4f(glGetUniformLocation(program, name), values[0], values[1], values[2], values[3]);
    glUseProgram(0);
}

void Shader::setUniformM4f(const char* name, const float* matrix) const {
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, matrix);
    glUseProgram(0);
}