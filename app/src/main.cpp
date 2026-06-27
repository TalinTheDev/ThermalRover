#include "app.hpp"
#include "renderer/renderer.hpp"

class ClearLayer : public AppLayer {
public:
  ClearLayer() : renderer() {}
  void update() override {}
  void render() override {
    renderer.clearColor(0.5f, 0.25f, 0.95f, 1.0f);
  }
  void transition() override {}

private:
  Renderer renderer;
};

int main() {
  App app(1000, 500, "Hello World!");

  app.newLayer<ClearLayer>();
  app.run();
}