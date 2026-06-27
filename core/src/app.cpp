#include "app.hpp"
#include "utils/logger.hpp"
#include "window/window.hpp"

#include <format>
#include <memory>
#include <string>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

Logger App::log;

App::App(int width, int height, const std::string &title) {
  log.info("App Initialized");

  glfwSetErrorCallback(errorCallback);
  if (!glfwInit()) {
    log.error("GLFW Initialization failed");
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window = std::make_unique<Window>(width, height, title, log);

  glfwMakeContextCurrent(window->get());
  gladLoadGL(glfwGetProcAddress);
};

App::~App() { glfwTerminate(); };

void App::run() {
  running = true;

  glfwSwapInterval(1);
  while (running) {
    int width, height;
    glfwGetFramebufferSize(window->get(), &width, &height);
    glViewport(0, 0, width, height);

    if (glfwWindowShouldClose(window->get())) {
      running = false;
      break;
    }
    glfwPollEvents();

    for (auto &layer : layerStack) {
      layer->update();
    }

    for (auto &layer : layerStack) {
      layer->render();
    }

    glfwSwapBuffers(window->get());
  }
};

void App::terminate() { glfwTerminate(); }

void App::errorCallback(int error, const char *description) {
  log.error(std::format("GLFW Error ({}): {}", error, description));
}