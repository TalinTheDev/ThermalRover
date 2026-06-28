#pragma once
#include "buffer.hpp"
#include "shader.hpp"
#include "types.hpp"

class Renderer2D {
public:
    Renderer2D();
    ~Renderer2D();

    void clearColor(Color color);
    void drawBuffer(Buffer& buffer, uint vertexCount);
    void useShader(Shader& shader);
};

