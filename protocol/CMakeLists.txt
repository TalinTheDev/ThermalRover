cmake_minimum_required(VERSION 4.2)

project(
    ThermalRoverProtocol
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
    src/protocol/udp.cpp
    src/protocol/lidar_imu_odometry.cpp
    src/protocol/motion_control.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

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
