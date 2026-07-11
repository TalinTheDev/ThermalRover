cmake_minimum_required(VERSION 4.2)

project(
    ThermalRover
    VERSION 1.0.0
    LANGUAGES CXX
)

# C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile Commands generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Sub Modules
add_subdirectory(core)
add_subdirectory(app)
add_subdirectory(protocol)