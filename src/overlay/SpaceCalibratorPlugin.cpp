#include "SpaceCalibratorPlugin.h"

#include "Icons.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "SpaceCalibratorUmbrellaRuntime.h"
#include "UserInterface.h"
#include "Widgets.h"

#include <imgui.h>

#include <memory>

extern SCIPCClient Driver;

const char *SpaceCalibratorPlugin::IconGlyph() const
{
	return ICON_PAIR_CALIBRATE;
}

void SpaceCalibratorPlugin::OnStart(openvr_pair::overlay::ShellContext &)
{
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
	openvr_pair::ui::Card("Calibration", "Existing calibration controls", []() {
		CCal_DrawTab();
	});
}

bool SpaceCalibratorPlugin::IpcStatusOk(openvr_pair::overlay::ShellContext &) const
{
	return Driver.IsConnected();
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateSpaceCalibratorPlugin()
{
	return std::make_unique<SpaceCalibratorPlugin>();
}

} // namespace openvr_pair::overlay
