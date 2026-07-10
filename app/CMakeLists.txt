cmake_minimum_required(VERSION 4.2)

project(
    ThermalRoverApp
    VERSION 1.0.0
    LANGUAGES CXX
)

# C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile Commands generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Executable
add_executable(${PROJECT_NAME}
    src/main.cpp
 "src/network.cpp")

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ThermalRoverCore)

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