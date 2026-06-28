#pragma once
#include "renderer/renderer2d.hpp"
#include "utils/logger.hpp"
#include "window/window.hpp"
#include <memory>
#include <string>
#include <vector>

class AppLayer {
public:
    virtual ~AppLayer() = default;

    virtual void update() = 0;
    virtual void render() = 0;

protected:
    static Renderer2D renderer;
};

class App {
public:
    App(int width, int height, const std::string& title);
    ~App();

    static Logger log;

    void run();
    void terminate();

    static float time();

    template <typename t, typename... Args> void newLayer(Args &&...args) {
        layerStack.emplace_back(std::make_unique<t>(std::forward<Args>(args)...));
    }

private:
    bool running = true;
    std::vector<std::unique_ptr<AppLayer>> layerStack;
    std::unique_ptr<Window> window;

    static void errorCallback(int error, const char* description);
};