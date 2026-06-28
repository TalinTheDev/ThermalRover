#include "types.hpp"

Vertex::Vertex(Coordinate3D coordinate) : coor(coordinate) {};
Vertex::Vertex(Coordinate3D coordinate, Color color) : coor(coordinate), c(color), colorSet(true) {};
Vertex::Vertex(Coordinate3D coordinate, Color color, Coordinate2D texCoordinate) : coor(coordinate), c(color), colorSet(true), tCoor(texCoordinate), texSet(true) {};

Vertex::~Vertex() {};

Coordinate3D& Vertex::coordinate() {
    return coor;
}

Vertex& Vertex::color(Color color) {
    this->c = color;
    colorSet = true;

    return *this;
}

Color& Vertex::color() {
    return c;
}

bool Vertex::hasColor() const {
    return colorSet;
}

Vertex& Vertex::tex(Coordinate2D coordinate) {
    this->tCoor = coordinate;
    texSet = true;

    return *this;
}

Coordinate2D& Vertex::tex() {
    return tCoor;
}

bool Vertex::hasTex() const {
    return texSet;
}