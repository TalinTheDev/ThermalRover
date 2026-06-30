#include "app.hpp"
#include "renderer/buffer.hpp"
#include "renderer/renderer2d.hpp"
#include "renderer/shader.hpp"
#include "types.hpp"
#include <cstdarg>
#include <imgui.h>
#include <layers/debug.hpp>
#include <memory>
#include <renderer/framebuffer.hpp>
#include <vector>


App app(1300, 900, "Hello World!");

class ClearLayer : public AppLayer {
public:
    ClearLayer() {};
    void renderUI() override {}
    void update() override {}
    void render() override { App::getRenderer().clearColor({0.20f, 0.61f, 0.92f, 1.0f}); }
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

    void renderUI() override {}
    void render() override {
        triangleShader->setUniformf("time", App::time());
        App::getRenderer().useShader(*triangleShader);
        App::getRenderer().drawBuffer(*triangleBuffer, 3);
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
    void renderUI() override {}
    void render() override {
        rectangleShader->setUniformf("time", App::time() * 2);
        App::getRenderer().useShader(*rectangleShader);
        App::getRenderer().drawBuffer(*rectangleBuffer, 6);
    }

private:
    std::vector<Vertex> vertices;
    std::unique_ptr<Buffer> rectangleBuffer;
    std::unique_ptr<Shader> rectangleShader;
};

void test() {
    ImGui::Begin("Test Window");
    ImVec2 size = ImGui::GetContentRegionAvail();
    app.getFB().resize({size.x, size.y});
    ImGui::Image(
    (ImTextureID)(uintptr_t)app.getFB().getTexture(),
        size,
        ImVec2(0, 1),   // flip vertically
        ImVec2(1, 0));
    ImGui::End();
}

int main() {
    app.useFramebuffer();

    app.newLayer<ClearLayer>();
    app.newLayer<TriangleLayer>();
    app.newLayer<RectangleLayer>();
    app.newLayer<DebugLayer>(app.getWindow()).push(test);

    app.run();
}