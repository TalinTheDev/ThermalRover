#pragma once

#include "imgui.h"

namespace ui {
struct ThemeColors {
    ImVec4 healthy;
    ImVec4 warning;
    ImVec4 error;
    ImVec4 accent;
};

void applyTheme(int theme);
const ThemeColors& themeColors();
// Emphasize only the main action and the emergency stop.
bool primaryButton(const char* label, ImVec2 size = {0.0f, 0.0f});
bool dangerButton(const char* label, ImVec2 size = {0.0f, 0.0f});
} // namespace ui
