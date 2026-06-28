#pragma once
#include "utils/logger.hpp"
#include <string>
#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

class Window {
public:
    Window(int width, int height, const std::string& title, Logger& log);
    ~Window();

    GLFWwindow* get();

private:
    GLFWwindow* id;
};