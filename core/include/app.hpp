#pragma once
#include "renderer/renderer2d.hpp"
#include "utils/logger.hpp"
#include "window/window.hpp"
#include <memory>
#include <string>
#include <vector>
#include <renderer/framebuffer.hpp>

class AppLayer {
public:
    virtual ~AppLayer() = default;

    virtual void update() = 0;
    virtual void render() = 0;
    virtual void renderUI() = 0;
};

class App {
public:
    App(int width, int height, const std::string& title);
    ~App();

    static Logger log;

    void run();
    void terminate();

    static float time();

    template <typename t, typename... Args> t& newLayer(Args &&...args) {
        std::unique_ptr<t> layer = std::make_unique<t>(std::forward<Args>(args)...);
        t& ref = *layer;
        layerStack.push_back(std::move(layer));
        return ref;
    }

    Window& getWindow();
    FrameBuffer& getFB();

    static Renderer2D& getRenderer();


    void useFramebuffer();
    void useScreenbuffer();


    void setRenderSize(Vector2 size);
private:
    bool running = true;
    bool usingFB = false;
    std::vector<std::unique_ptr<AppLayer>> layerStack;
    std::unique_ptr<Window> window;
    std::unique_ptr<FrameBuffer> fb;
    static Renderer2D renderer;
    Vector2 renderSize;

    static void errorCallback(int error, const char* description);
};