#include "app.hpp"
#include "renderer/buffer.hpp"
#include "renderer/renderer2d.hpp"
#include "renderer/shader.hpp"
#include "types.hpp"
#include <memory>
#include <vector>


class ClearLayer : public AppLayer {
public:
    ClearLayer() {};
    void update() override {}
    void render() override { renderer.clearColor({0.20f, 0.61f, 0.92f, 1.0f}); }
};

class TriangleLayer : public AppLayer {
public:
    TriangleLayer() {
        vertices.emplace_back(Coordinate3D{-0.75f,0.75f, 0.0f}, Color{1.0f, 0.0f, 0.0f, 1.0f});
        vertices.emplace_back(Coordinate3D{-0.75f,-0.75f, 0.0f}, Color{0.0f, 1.0f, 0.0f, 1.0f});
        vertices.emplace_back(Coordinate3D{0.0f,-0.75f, 0.0f}, Color{0.0f, 0.0f, 1.0f, 1.0f});

        triangleBuffer = std::make_unique<Buffer>(false);
        triangleBuffer->data(vertices);
        triangleBuffer->shaderData(0, 3, tFloat, 7);
        triangleBuffer->shaderData(1, 4, tFloat, 7, 3);
        triangleShader = std::make_unique<Shader>("../../../../app/res/shaders/triangle/basic.vert", "../../../../app/res/shaders/triangle/basic.frag");
    }

    void update() override {}

    void render() override {
        triangleShader->setUniformf("time", App::time());
        renderer.useShader(*triangleShader);
        renderer.drawBuffer(*triangleBuffer, 3);
    }

private:
    std::vector<Vertex> vertices;
    std::unique_ptr<Buffer> triangleBuffer;
    std::unique_ptr<Shader> triangleShader;
};

class RectangleLayer : public AppLayer {
public:
    RectangleLayer() {
        vertices.emplace_back(Coordinate3D{0.0f,0.75f, 0.0f}, Color{1.0f, 0.0f, 0.0f, 1.0f});
        vertices.emplace_back(Coordinate3D{0.0f,-0.75f, 0.0f}, Color{0.0f, 1.0f, 0.0f, 1.0f});
        vertices.emplace_back(Coordinate3D{0.75f,-0.75f, 0.0f}, Color{0.0f, 0.0f, 1.0f, 1.0f});
        vertices.emplace_back(Coordinate3D{0.75f,0.75f, 0.0f}, Color{1.0f, 1.0f, 1.0f, 1.0f});

        rectangleBuffer = std::make_unique<Buffer>(true);
        rectangleBuffer->data(vertices, {0, 1, 2, 2, 3, 0});
        rectangleBuffer->shaderData(0, 3, tFloat, 7);
        rectangleBuffer->shaderData(1, 4, tFloat, 7, 3);
        rectangleShader = std::make_unique<Shader>("../../../../app/res/shaders/triangle/basic.vert", "../../../../app/res/shaders/triangle/basic.frag");
    }

    void update() override {}

    void render() override {
        rectangleShader->setUniformf("time", App::time() * 2);
        renderer.useShader(*rectangleShader);
        renderer.drawBuffer(*rectangleBuffer, 6);
    }

private:
    std::vector<Vertex> vertices;
    std::unique_ptr<Buffer> rectangleBuffer;
    std::unique_ptr<Shader> rectangleShader;
};

int main() {
    App app(1300, 900, "Hello World!");

    app.newLayer<ClearLayer>();
    app.newLayer<TriangleLayer>();
    app.newLayer<RectangleLayer>();

    app.run();
}