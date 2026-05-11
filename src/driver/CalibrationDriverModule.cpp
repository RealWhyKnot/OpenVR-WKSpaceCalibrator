#include "DriverModule.h"
#include "FeatureFlags.h"
#include "ServerTrackedDeviceProvider.h"

namespace calibration {
namespace {

class CalibrationDriverModule final : public DriverModule
{
public:
	const char *Name() const override { return "Space Calibrator"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureCalibration; }
	const char *PipeName() const override { return OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME; }

	bool Init(DriverModuleContext &context) override
	{
		provider_ = context.provider;
		return provider_ != nullptr;
	}

	void Shutdown() override
	{
		provider_ = nullptr;
	}

	bool HandleRequest(const protocol::Request &request, protocol::Response &response) override
	{
		if (!provider_) return false;
		switch (request.type) {
		case protocol::RequestSetDeviceTransform:
			provider_->SetDeviceTransform(request.setDeviceTransform);
			response.type = protocol::ResponseSuccess;
			return true;
		case protocol::RequestDebugOffset:
			provider_->HandleApplyRandomOffset();
			response.type = protocol::ResponseSuccess;
			return true;
		case protocol::RequestSetAlignmentSpeedParams:
			provider_->HandleSetAlignmentSpeedParams(request.setAlignmentSpeedParams);
			response.type = protocol::ResponseSuccess;
			return true;
		case protocol::RequestSetTrackingSystemFallback:
			provider_->SetTrackingSystemFallback(request.setTrackingSystemFallback);
			response.type = protocol::ResponseSuccess;
			return true;
		default:
			return false;
		}
	}

private:
	ServerTrackedDeviceProvider *provider_ = nullptr;
};

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
	return std::make_unique<CalibrationDriverModule>();
}

} // namespace calibration
