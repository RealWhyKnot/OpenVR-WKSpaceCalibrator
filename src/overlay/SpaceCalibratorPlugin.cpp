#include "SpaceCalibratorPlugin.h"

#include "Protocol.h"
#include "ShellContext.h"
#include "SpaceCalibratorUmbrellaRuntime.h"
#include "UserInterface.h"

#include <imgui.h>

#include <memory>

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
	CCal_DrawTab();
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateSpaceCalibratorPlugin()
{
	return std::make_unique<SpaceCalibratorPlugin>();
}

} // namespace openvr_pair::overlay
