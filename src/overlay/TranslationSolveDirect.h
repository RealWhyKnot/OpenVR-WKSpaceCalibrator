#pragma once

#include <Eigen/Geometry>
#include <vector>

#include "CalibrationCalc.h"  // for Sample

namespace spacecal::translation {

struct DirectOptions {
    bool useTukeyBiweight = false;
    bool useVelocityAwareWeighting = false;
};

struct DirectResult {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d offset      = Eigen::Vector3d::Zero();  // S, target offset in ref-local frame
    double residualRms          = 0.0;
    double conditionRatio       = 0.0;
    int iterations              = 0;
};

DirectResult SolveDirect(
    const std::vector<Sample>& samples,
    const Eigen::Matrix3d& C_R,
    const DirectOptions& opts);

DirectResult CalibrateTranslationUpstream(
    const std::vector<Sample>& samples,
    const Eigen::Matrix3d& C_R);

} // namespace spacecal::translation
