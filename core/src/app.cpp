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

  while (running) {
    if (glfwWindowShouldClose(window->get())) {
      running = false;
      break;
    } else {
      glfwPollEvents();
    }

    for (auto &layer : layerStack) {
      layer->update();
    }

    for (auto &layer : layerStack) {
      layer->render();
    }
  }
};

template <typename T, typename... Args> void App::newLayer(Args &&...args) {
  T *layer = new T(std::forward<Args>(args)...);
  layerStack.emplace_back(layer);
}

void App::errorCallback(int error, const char *description) {
  log.error(std::format("GLFW Error ({}): {}", error, description));
}