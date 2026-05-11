#pragma once

#include "FeaturePlugin.h"
#include "Protocol.h"

class SpaceCalibratorPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	const char *Name() const override { return "Space Calibrator"; }
	const char *FlagFileName() const override { return "enable_calibration.flag"; }
	const char *PipeName() const override { return OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME; }

	void OnStart(openvr_pair::overlay::ShellContext &context) override;
	void OnShutdown(openvr_pair::overlay::ShellContext &context) override;
	void Tick(openvr_pair::overlay::ShellContext &context) override;
	void DrawTab(openvr_pair::overlay::ShellContext &context) override;
};
