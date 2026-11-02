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

# Keep fonts beside each executable, independent of its working directory.
function(thermal_rover_bundle_fonts target)
    target_compile_definitions(${target} PRIVATE
        THERMAL_ROVER_FONT_PATH="${CMAKE_SOURCE_DIR}/core/vendor/imgui/misc/fonts/Karla-Regular.ttf")
    if(CMAKE_GENERATOR MATCHES "^Visual Studio")
        # Resolve at build time so /p:OutDir also relocates the assets.
        set(font_output_directory "$(OutDir)fonts")
    else()
        set(font_output_directory "$<TARGET_FILE_DIR:${target}>/fonts")
    endif()
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${font_output_directory}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/core/vendor/imgui/misc/fonts/Karla-Regular.ttf"
            "${font_output_directory}/Karla-Regular.ttf"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/core/vendor/imgui/docs/FONTS.md"
            "${font_output_directory}/FONT_CREDITS.md"
        VERBATIM)
endfunction()

# Sub Modules
add_subdirectory(core)
add_subdirectory(app)
add_subdirectory(protocol)