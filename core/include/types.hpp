#pragma once
#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

struct Coordinate3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Coordinate2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct Color {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 0.0f;
};

class Vertex {
public:
    Vertex(Coordinate3D coordinate);
    Vertex(Coordinate3D coordinate, Color color);
    Vertex(Coordinate3D coordinate, Color color, Coordinate2D texCoordinate);
    ~Vertex();

    Coordinate3D& coordinate();

    Vertex& color(Color color);
    Color& color();
    bool hasColor() const;

    Vertex& tex(Coordinate2D coordinate);
    Coordinate2D& tex();
    bool hasTex() const;

private:
    Coordinate3D coor;
    Color c;
    bool colorSet = false;
    Coordinate2D tCoor;
    bool texSet = false;
};

typedef unsigned int uint;
typedef GLenum DataType;

const int tFloat = GL_FLOAT;