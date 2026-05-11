#pragma once

#include <imgui/imgui.h>

namespace spacecal::ui {

void DrawStatusDot(ImU32 color, float radiusScale = 0.32f);
void ShowVersionLine();
void GetModeStatus(const char*& label, const char*& tooltip, ImVec4& accent);

} // namespace spacecal::ui
