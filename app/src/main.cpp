#include "network.hpp"
#include <app.hpp>
#include <thread>


void runApp() {
    App app(1500, 1000, "Networking stack");
    app.run();
}

int main() {
    Server server;
    if(!server.alive) {
        return 1;
    }

    std::thread appThread(runApp);
    appThread.join();
    return 0;
}
