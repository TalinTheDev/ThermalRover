#include "renderer/renderer.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

Renderer::Renderer(){

};
Renderer::~Renderer(){};

void Renderer::clearColor(float r, float g, float b, float a){
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(r, g, b, a);
}