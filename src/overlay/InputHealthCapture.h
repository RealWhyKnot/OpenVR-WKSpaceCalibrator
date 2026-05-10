#pragma once

#include "InputHealthSnapshotReader.h"

#include <fstream>
#include <string>

class InputHealthCaptureWriter
{
public:
	void Capture(const InputHealthSnapshotReader &reader, bool monitoringEnabled, double now, bool force);

	const std::string &Path() const { return path_; }
	const std::string &LastError() const { return last_error_; }

private:
	void EnsureOpen();

	std::ofstream file_;
	std::string path_;
	std::string last_error_;
	double next_capture_time_ = 0.0;
};
