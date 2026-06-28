#include "renderer/renderer.hpp"
#include "renderer/buffer.hpp"
#include "renderer/shader.hpp"
#include "types.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

Renderer::Renderer() {

};
Renderer::~Renderer() {};

void Renderer::clearColor(Color color) {
  glClear(GL_COLOR_BUFFER_BIT);
  glClearColor(color.r, color.g, color.b, color.a);
}

void Renderer::drawBuffer(Buffer &buffer, uint vertexCount) {
  buffer.bind();

  if (buffer.isIndex())
    glDrawElements(GL_TRIANGLES, vertexCount, GL_UNSIGNED_INT, 0);
  else
    glDrawArrays(GL_TRIANGLES, 0, vertexCount);

  buffer.unbind();
}

void Renderer::useShader(Shader& shader){
  glUseProgram(shader);
}