#include "utils/logger.hpp"
#include "window/window.hpp"
#include <string>
#include <GLFW/glfw3.h>


Window::Window(int width, int height, const std::string& title, Logger& log) {
    GLFWwindow* window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);

    if(!window) {
        log.error("GLFW Window Creation Failed");
    } else {
        log.info("GLFW Window created");
    }

    id = window;
};

Window::~Window() {
    if(id != nullptr) {
        glfwDestroyWindow(id);
    }
};

GLFWwindow* Window::get() {
    return id;
}
