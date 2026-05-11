#include "SpaceCalibratorPlugin.h"

#include "EmbeddedFiles.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "SpaceCalibratorUmbrellaRuntime.h"
#include "UserInterface.h"

#include <imgui.h>

#include <memory>

void SpaceCalibratorPlugin::OnStart(openvr_pair::overlay::ShellContext &)
{
	// Match the standalone SpaceCalibrator binary's typography so the
	// calibration UI looks the way long-time users expect. The umbrella
	// shell otherwise falls through to ImGui's default ProggyClean, which
	// renders too small and too pixelated for this overlay's size.
	auto &io = ImGui::GetIO();
	if (io.Fonts->Fonts.empty() ||
		(io.Fonts->Fonts.size() == 1 && io.Fonts->Fonts[0] == io.FontDefault &&
		 io.FontDefault != nullptr && io.FontDefault->FontSize < 18.0f))
	{
		io.Fonts->AddFontFromMemoryCompressedTTF(
			DroidSans_compressed_data,
			DroidSans_compressed_size,
			24.0f);
	}

	CCal_SetInUmbrella(true);
	CCal_UmbrellaStart();
}

void SpaceCalibratorPlugin::OnShutdown(openvr_pair::overlay::ShellContext &)
{
	CCal_UmbrellaShutdown();
}

void SpaceCalibratorPlugin::Tick(openvr_pair::overlay::ShellContext &)
{
	CCal_UmbrellaTick();
}

void SpaceCalibratorPlugin::DrawTab(openvr_pair::overlay::ShellContext &)
{
	CCal_DrawTab();
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateSpaceCalibratorPlugin()
{
	return std::make_unique<SpaceCalibratorPlugin>();
}

} // namespace openvr_pair::overlay
