#pragma once

#include "Calibration.h"
#include <cstddef>
#include <iosfwd>

#ifdef _WIN32
#include <windows.h>  // DWORD for StripRegistryNullTerminator's signature
#endif

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx);

// Set to true if the last LoadProfile call found a chaperone geometry array
// whose length was not a multiple of 12 (corrupt profile). The UI uses this
// to show a banner so the user knows auto-apply is disabled.
extern bool g_chaperoneGeometrySizeMismatch;

// Stream-based serialization: registry-free counterparts to LoadProfile /
// SaveProfile, exposed for unit-testing the schema migration + round-trip
// without touching the Windows registry. Not used by production code (the
// overlay's hot path goes through LoadProfile / SaveProfile, which call
// these internally after wrapping a stringstream around the registry value).
void ParseProfile(CalibrationContext &ctx, std::istream &stream);
void WriteProfile(CalibrationContext &ctx, std::ostream &out);

#ifdef _WIN32
// Strip the trailing null terminator from a RegGetValueA byte count for
// REG_SZ. Returns 0 if reportedSize == 0 (regression guard against the
// underflow bug fixed 2026-05-04). See Configuration.cpp::ReadRegistryKey
// for the full failure mode.
size_t StripRegistryNullTerminator(DWORD reportedSize);
#endif
