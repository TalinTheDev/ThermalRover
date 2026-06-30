#pragma once
#include "types.hpp"

#include <vector>

class FrameBuffer {
public:
    FrameBuffer();
    ~FrameBuffer();

    void bind() const;
    void unbind() const;

    uint getTexture();

    void resize(Vector2 size);

private:
    uint FBO = 0, texture = 0;
};