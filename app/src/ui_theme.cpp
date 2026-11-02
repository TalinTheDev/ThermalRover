#include "ui_theme.hpp"

namespace ui {
namespace {
constexpr ImVec4 rgb(int r, int g, int b, float a = 1.0f) {
    return {r / 255.0f, g / 255.0f, b / 255.0f, a};
}
ThemeColors activeColors{};
ImVec4 primary{}, primaryHovered{}, primaryActive{};
} // namespace

const ThemeColors& themeColors() { return activeColors; }

void applyTheme(int theme) {
    auto& style = ImGui::GetStyle();
    // Geometry is shared across themes so switching colors never moves controls.
    ImGui::StyleColorsDark(&style);
    style.WindowRounding = 0.0f;
    style.ChildRounding = 3.0f;
    style.FrameRounding = 3.0f;
    style.PopupRounding = 4.0f;
    style.GrabRounding = 2.0f;
    style.ScrollbarRounding = 3.0f;
    style.WindowBorderSize = 0.0f;
    style.ChildBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.WindowPadding = {16.0f, 14.0f};
    style.FramePadding = {10.0f, 6.0f};
    style.ItemSpacing = {10.0f, 8.0f};
    style.ItemInnerSpacing = {8.0f, 5.0f};
    style.CellPadding = {8.0f, 5.0f};
    style.IndentSpacing = 16.0f;
    style.ScrollbarSize = 12.0f;
    style.GrabMinSize = 10.0f;
    style.DisabledAlpha = 0.50f;
    style.ButtonTextAlign = {0.5f, 0.5f};
    style.SelectableTextAlign = {0.0f, 0.5f};
    style.SeparatorTextBorderSize = 1.0f;
    style.SeparatorTextPadding = {0.0f, 8.0f};

    const bool light = theme == 1;
    const bool neutral = theme == 2;
    const auto window = light ? rgb(234, 237, 240) : neutral ? rgb(27, 28, 30) : rgb(26, 32, 39);
    const auto surface = light ? rgb(250, 251, 252) : neutral ? rgb(34, 35, 38) : rgb(33, 40, 48);
    const auto field = light ? rgb(255, 255, 255) : neutral ? rgb(27, 28, 31) : rgb(26, 33, 41);
    const auto border = light ? rgb(205, 211, 217) : neutral ? rgb(60, 62, 66) : rgb(56, 66, 77);
    const auto text = light ? rgb(32, 40, 49) : rgb(228, 232, 237);
    const auto muted = light ? rgb(91, 102, 114) : rgb(158, 169, 182);
    const auto button = light ? rgb(237, 240, 244) : neutral ? rgb(45, 47, 51) : rgb(43, 53, 65);
    const auto hover = light ? rgb(220, 228, 237) : neutral ? rgb(58, 61, 66) : rgb(55, 69, 84);
    const auto pressed = light ? rgb(205, 218, 232) : neutral ? rgb(67, 71, 77) : rgb(65, 82, 100);
    activeColors = light
        ? ThemeColors{rgb(28, 111, 73), rgb(139, 90, 14), rgb(177, 44, 39), rgb(36, 99, 160)}
        : ThemeColors{rgb(122, 203, 160), rgb(231, 187, 105), rgb(246, 142, 137), rgb(137, 185, 233)};
    primary = rgb(39, 85, 128);
    primaryHovered = rgb(48, 100, 147);
    primaryActive = rgb(31, 69, 106);

    auto* c = style.Colors;
    c[ImGuiCol_Text] = text;
    c[ImGuiCol_TextDisabled] = muted;
    c[ImGuiCol_WindowBg] = window;
    c[ImGuiCol_ChildBg] = surface;
    c[ImGuiCol_PopupBg] = surface;
    c[ImGuiCol_Border] = border;
    c[ImGuiCol_BorderShadow] = rgb(0, 0, 0, 0);
    c[ImGuiCol_FrameBg] = field;
    c[ImGuiCol_FrameBgHovered] = button;
    c[ImGuiCol_FrameBgActive] = hover;
    c[ImGuiCol_Button] = button;
    c[ImGuiCol_ButtonHovered] = hover;
    c[ImGuiCol_ButtonActive] = pressed;
    c[ImGuiCol_Header] = button;
    c[ImGuiCol_HeaderHovered] = hover;
    c[ImGuiCol_HeaderActive] = pressed;
    c[ImGuiCol_Separator] = border;
    c[ImGuiCol_SeparatorHovered] = activeColors.accent;
    c[ImGuiCol_SeparatorActive] = activeColors.accent;
    c[ImGuiCol_CheckMark] = activeColors.accent;
    c[ImGuiCol_SliderGrab] = activeColors.accent;
    c[ImGuiCol_SliderGrabActive] = activeColors.accent;
    c[ImGuiCol_ScrollbarBg] = surface;
    c[ImGuiCol_ScrollbarGrab] = border;
    c[ImGuiCol_ScrollbarGrabHovered] = muted;
    c[ImGuiCol_ScrollbarGrabActive] = activeColors.accent;
    c[ImGuiCol_TableHeaderBg] = button;
    c[ImGuiCol_TableBorderStrong] = border;
    c[ImGuiCol_TableBorderLight] = border;
    c[ImGuiCol_TableRowBg] = rgb(0, 0, 0, 0);
    c[ImGuiCol_TableRowBgAlt] = light ? rgb(0, 0, 0, 0.025f) : rgb(255, 255, 255, 0.025f);
    c[ImGuiCol_TextSelectedBg] = light ? rgb(102, 156, 213, 0.3f) : rgb(96, 149, 206, 0.3f);
    c[ImGuiCol_PlotHistogram] = activeColors.healthy;
    c[ImGuiCol_NavCursor] = activeColors.accent;
}

bool primaryButton(const char* label, ImVec2 size) {
    ImGui::PushStyleColor(ImGuiCol_Button, primary);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, primaryHovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, primaryActive);
    ImGui::PushStyleColor(ImGuiCol_Text, rgb(255, 255, 255));
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return pressed;
}

bool dangerButton(const char* label, ImVec2 size) {
    ImGui::PushStyleColor(ImGuiCol_Button, rgb(157, 43, 39));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, rgb(177, 52, 46));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, rgb(128, 32, 29));
    ImGui::PushStyleColor(ImGuiCol_Text, rgb(255, 255, 255));
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return pressed;
}
} // namespace ui
