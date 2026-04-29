#pragma once

#include "Calibration.h"
#include <iosfwd>

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx);

// Stream-based serialization: registry-free counterparts to LoadProfile /
// SaveProfile, exposed for unit-testing the schema migration + round-trip
// without touching the Windows registry. Not used by production code (the
// overlay's hot path goes through LoadProfile / SaveProfile, which call
// these internally after wrapping a stringstream around the registry value).
void ParseProfile(CalibrationContext &ctx, std::istream &stream);
void WriteProfile(CalibrationContext &ctx, std::ostream &out);
