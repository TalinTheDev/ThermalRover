#pragma once
#include "types.hpp"

#include <vector>

class Buffer {
public:
    Buffer(bool indices = false);
    ~Buffer();

    void bind() const;
    void unbind() const;
    void data(std::vector<Vertex> vertices, std::vector<uint> indices = {}) const;
    void shaderData(uint location, uint size, DataType type, uint stride,
                    uint offset = 0) const;

    bool isIndex() const;

private:
    uint VAO = 0, VBO = 0, EBO = 0;
};