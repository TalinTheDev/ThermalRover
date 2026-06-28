cmake_minimum_required(VERSION 4.2)
include(FetchContent)

project(
    ThermalRoverCore
    VERSION 1.0.0
    LANGUAGES CXX
)

# C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile Commands generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Library
add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME} PRIVATE
    src/utils/logger.cpp
    src/window/window.cpp
    "src/renderer/renderer2d.cpp"
    src/renderer/buffer.cpp
    src/types.cpp
    src/app.cpp
    src/renderer/shader.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Vendored Libraries
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glm)

add_subdirectory(vendor/glad)

target_link_libraries(${PROJECT_NAME}
    PUBLIC glfw
    PUBLIC glad
    PUBLIC  glm::glm
)

# Warnings
if(MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            /W4
            /permissive-
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow
    )
endif()