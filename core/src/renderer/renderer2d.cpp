#include "renderer/buffer.hpp"
#include "renderer/renderer2d.hpp"
#include "renderer/shader.hpp"
#include "types.hpp"
#include <glad/gl.h>

Renderer2D::Renderer2D() {

};
Renderer2D::~Renderer2D() {};

void Renderer2D::clearColor(Color color) {
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(color.r, color.g, color.b, color.a);
}

void Renderer2D::drawBuffer(Buffer& buffer, uint vertexCount) {
    buffer.bind();

    if(buffer.isIndex())
        glDrawElements(GL_TRIANGLES, vertexCount, GL_UNSIGNED_INT, 0);
    else
        glDrawArrays(GL_TRIANGLES, 0, vertexCount);

    buffer.unbind();
}

void Renderer2D::useShader(Shader& shader) {
    glUseProgram(shader);
}