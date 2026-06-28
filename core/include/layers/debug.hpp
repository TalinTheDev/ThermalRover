#pragma once
#include "app.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "window/window.hpp"
#include <vector>

class DebugLayer : public AppLayer {
public:
    DebugLayer(Window& window);
    ~DebugLayer();

    void update() override;
    void render() override;

    DebugLayer& push(void(*layer)());

private:
    std::vector<void(*)()> debugStack;
};