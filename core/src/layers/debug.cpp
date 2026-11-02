#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "layers/debug.hpp"
#include "window/window.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <imgui_internal.h>

DebugLayer::DebugLayer(Window& window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui_ImplGlfw_InitForOpenGL(window.get(), true);
    ImGui_ImplOpenGL3_Init();

    ImGuiStyle& style = ImGui::GetStyle();
    style.FontSizeBase = 17;
    style.WindowRounding = 0.0f;
    style.ChildRounding = 0.0f;
    style.FrameRounding = 0.0f;
    style.PopupRounding = 0.0f;
    style.ScrollbarRounding = 0.0f;
    style.GrabRounding = 0.0f;
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.FramePadding = {11.0f, 8.0f};
    style.WindowPadding = {18.0f, 16.0f};
    style.ItemSpacing = {13.0f, 11.0f};
    style.ItemInnerSpacing = {10.0f, 8.0f};
    ImGui::StyleColorsDark();
    style.Colors[ImGuiCol_WindowBg] = {0.025f, 0.029f, 0.035f, 1.0f};
    style.Colors[ImGuiCol_ChildBg] = {0.045f, 0.050f, 0.058f, 1.0f};
    style.Colors[ImGuiCol_Border] = {0.20f, 0.22f, 0.25f, 1.0f};
    style.Colors[ImGuiCol_FrameBg] = {0.075f, 0.082f, 0.092f, 1.0f};
    style.Colors[ImGuiCol_FrameBgHovered] = {0.12f, 0.13f, 0.15f, 1.0f};
    style.Colors[ImGuiCol_FrameBgActive] = {0.16f, 0.17f, 0.19f, 1.0f};
    style.Colors[ImGuiCol_Button] = {0.14f, 0.15f, 0.17f, 1.0f};
    style.Colors[ImGuiCol_ButtonHovered] = {0.24f, 0.26f, 0.29f, 1.0f};
    style.Colors[ImGuiCol_ButtonActive] = {0.30f, 0.32f, 0.35f, 1.0f};
    style.Colors[ImGuiCol_Header] = {0.14f, 0.15f, 0.17f, 1.0f};
    style.Colors[ImGuiCol_HeaderHovered] = {0.22f, 0.24f, 0.27f, 1.0f};
    style.Colors[ImGuiCol_HeaderActive] = {0.28f, 0.30f, 0.33f, 1.0f};
}

DebugLayer::~DebugLayer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void DebugLayer::update() {}
void DebugLayer::render() {}
void DebugLayer::renderUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    for(void(*func)() : debugStack) {
        func();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    ImGuiIO& io = ImGui::GetIO();
    if(io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow* context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();

        glfwMakeContextCurrent(context);
    }
}

DebugLayer& DebugLayer::push(void(*layer)()) {
    debugStack.push_back(layer);

    return *this;
}
