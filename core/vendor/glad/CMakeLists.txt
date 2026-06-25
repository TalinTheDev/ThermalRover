cmake_minimum_required(VERSION 4.2)

project(glad)

# Library
add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME} PRIVATE src/gl.c)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)