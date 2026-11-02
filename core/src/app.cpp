#include "app.hpp"
#include "utils/logger.hpp"
#include "window/window.hpp"
#include <format>
#include <memory>
#include <renderer/renderer2d.hpp>
#include <string>
#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <renderer/framebuffer.hpp>
#include <types.hpp>

Logger App::log;
Renderer2D App::renderer;

App::App(int width, int height, const std::string& title, bool visible) {
    log.info("App Initialized");

    glfwSetErrorCallback(errorCallback);
    if(!glfwInit()) {
        log.error("GLFW Initialization failed");
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);

    window = std::make_unique<Window>(width, height, title, log);
    if(window->get() == nullptr) {
        return;
    }

    glfwMakeContextCurrent(window->get());
    initialized = gladLoadGL(glfwGetProcAddress) != 0;
    if(!initialized) {
        log.error("OpenGL loader initialization failed");
    }
};

App::~App() {
    shutdown();
}

void App::shutdown() {
    if(shutdownComplete) return;
    layerStack.clear();
    fb.reset();
    window.reset();
    glfwTerminate();
    initialized = false;
    running = false;
    shutdownComplete = true;
}

void App::run() {
    if(!initialized || !window) {
        return;
    }
    running = true;

    glfwSwapInterval(1);
    while(running) {
        int width, height;
        glfwGetFramebufferSize(window->get(), &width, &height);
        renderSize.x = width;
        renderSize.y = height;
        if(!usingFB)  {
            glViewport(0, 0, width, height);
        }

        if(glfwWindowShouldClose(window->get())) {
            running = false;
            break;
        }
        glfwPollEvents();

        for(auto& layer : layerStack) {
            layer->update();
        }

        if(usingFB) {
            fb->bind();
        }
        glClearColor(0.035f, 0.045f, 0.060f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        for(auto& layer : layerStack) {
            layer->render();
        }
        if(usingFB) {
            fb->unbind();
        }
        for(auto& layer : layerStack) {
            layer->renderUI();
        }

        glfwSwapBuffers(window->get());
    }
};

void App::terminate() {
    running = false;
    if(window != nullptr && window->get() != nullptr) {
        glfwSetWindowShouldClose(window->get(), GLFW_TRUE);
    }
}

void App::errorCallback(int error, const char* description) {
    log.error(std::format("GLFW Error ({}): {}", error, description));
}

float App::time() {
    return glfwGetTime();
}

Window& App::getWindow() {
    return *window.get();
}

Renderer2D& App::getRenderer() {
    return renderer;
}


FrameBuffer& App::getFB() {
    if(!usingFB) {
        App::log.error("Trying to use framebuffer rendering without buffer initialization");
    }
    return *fb.get();
}

bool App::isInitialized() const {
    return initialized;
}

void App::useFramebuffer() {
    if(!usingFB) {
        fb = std::make_unique<FrameBuffer>();
        fb->bind();
        glBindTexture(GL_TEXTURE_2D, fb->getTexture());
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, renderSize.x, renderSize.y, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glViewport(0, 0, renderSize.x, renderSize.y);
        usingFB = true;
    }
};
void App::useScreenbuffer() {
    if(usingFB) {
        fb->unbind();
        usingFB = false;
    }
};

void App::setRenderSize(Vector2 size) {
    renderSize.x = size.x;
    renderSize.y = size.y;
}

Vector2 App::getRenderSize() {
    return renderSize;
}
