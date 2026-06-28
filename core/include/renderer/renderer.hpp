#pragma once
#include "buffer.hpp"
#include "shader.hpp"
#include "types.hpp"

class Renderer{
    public:
        Renderer();
        ~Renderer();

        void clearColor(Color color);
        void drawBuffer(Buffer& buffer, uint vertexCount);
        void useShader(Shader& shader);
};

