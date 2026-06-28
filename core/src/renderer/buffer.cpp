#include "renderer/buffer.hpp"
#include "types.hpp"
#include <glad/gl.h>
#include <vector>
#include <format>
#include <app.hpp>

Buffer::Buffer(bool indices) {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    if(indices)
        glGenBuffers(1, &EBO);
}

Buffer::~Buffer() {
    unbind();
    if(VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    if(EBO != 0) {
        glDeleteBuffers(1, &EBO);
    }
}

void Buffer::bind() const {
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    if(EBO != 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
}

void Buffer::unbind() const {
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if(EBO != 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Buffer::data(std::vector<Vertex> vertices, std::vector<uint> indices) const {
    bind();

    std::vector<float> points{};
    points.reserve(vertices.size() * 3);

    for(Vertex v : vertices) {
        points.push_back(v.coordinate().x);
        points.push_back(v.coordinate().y);
        points.push_back(v.coordinate().z);


        if(v.hasColor()) {
            points.push_back(v.color().r);
            points.push_back(v.color().g);
            points.push_back(v.color().b);
            points.push_back(v.color().a);
        }
    }


    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(float),
                 points.data(), GL_STATIC_DRAW);

    if(EBO != 0 && indices.size() > 0)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                     indices.data(), GL_STATIC_DRAW);

    unbind();
}

void Buffer::shaderData(uint location, uint size, DataType type, uint stride,
                        uint offset) const {
    bind();

    glVertexAttribPointer(location, size, type, GL_FALSE, stride * sizeof(type),
                          (void*)(offset * sizeof(type)));
    glEnableVertexAttribArray(location);

    unbind();
}

bool Buffer::isIndex() const { return EBO != 0; }