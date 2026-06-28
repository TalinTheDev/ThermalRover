#include "app.hpp"
#include "renderer/buffer.hpp"
#include "renderer/renderer2d.hpp"
#include "renderer/shader.hpp"
#include "types.hpp"
#include <memory>
#include <vector>

#define GLFW_INCLUDE_NONE

class ClearLayer : public AppLayer {
public:
    ClearLayer() {};
    void update() override {}
    void render() override { renderer.clearColor({0.20f, 0.61f, 0.92f, 1.0f}); }
};

class TriangleLayer : public AppLayer {
public:
    TriangleLayer() {
        Vertex v1({0.0f,0.5f, 0.0f});
        v1.color({1.0f, 0.0f, 0.0f, 1.0f});
        vertices.push_back(v1);

        Vertex v2({-0.5f,-0.5f, 0.0f});
        v2.color({0.0f, 1.0f, 0.0f, 1.0f});
        vertices.push_back(v2);

        Vertex v3({0.5f,-0.5f, 0.0f});
        v3.color({0.0f, 0.0f, 1.0f, 1.0f});
        vertices.push_back(v3);

        triangleBuffer = std::make_unique<Buffer>();
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

int main() {
    App app(1000, 500, "Hello World!");

    app.newLayer<ClearLayer>();
    app.newLayer<TriangleLayer>();
    app.run();
}