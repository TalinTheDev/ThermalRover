#include "window/window.hpp"
#include "utils/logger.hpp"
#include <string>

#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>


Window::Window(int width, int height, const std::string& title, Logger& log){
    GLFWwindow *window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);

    if(!window){
        log.error("GLFW Window Creation Failed");
    } else {
        log.info("GLFW Window created");
    }

    id = window;
};

Window::~Window(){
    glfwDestroyWindow(id);
};

GLFWwindow* Window::get(){
    return id;
}