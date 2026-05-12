#include "TranslationSolveDirect.h"

#include "CalibrationMetrics.h"
#include "RobustScale.h"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace spacecal::translation {

namespace {

constexpr int    kMaxIters             = 5;
constexpr double kWeightChangeThreshold = 0.01;
constexpr double kMadFloor             = 1e-3;
constexpr double kCauchyTune           = 1.345;
// Velocity-aware constants match the pairwise path exactly.
constexpr double kVelocityKappa  = 2.0;
constexpr double kVelocityRefMps = 0.3;

} // namespace

DirectResult SolveDirect(
    const std::vector<Sample>& samples,
    const Eigen::Matrix3d& C_R,
    const DirectOptions& opts)
{
    DirectResult result;

    const std::size_t N = samples.size();
    if (N < 2) {
        return result;
    }

    std::vector<double> weights(N, 1.0);
    std::vector<double> prevWeights(N, 1.0);

    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    Eigen::Vector3d S = Eigen::Vector3d::Zero();

    // Pre-compute per-sample b_world_i = r_i - C_R * t_i.
    std::vector<Eigen::Vector3d> b_world(N);
    for (std::size_t i = 0; i < N; ++i) {
        b_world[i] = samples[i].ref.trans - C_R * samples[i].target.trans;
    }

    for (int iter = 0; iter < kMaxIters; ++iter) {
        double W = 0.0;
        Eigen::Matrix3d M    = Eigen::Matrix3d::Zero();
        Eigen::Vector3d sum_b    = Eigen::Vector3d::Zero();
        Eigen::Vector3d sum_AT_b = Eigen::Vector3d::Zero();

        for (std::size_t i = 0; i < N; ++i) {
            const double w = weights[i];
            W += w;
            M.noalias()       += w * samples[i].ref.rot;
            sum_b.noalias()   += w * b_world[i];
            sum_AT_b.noalias() += w * (samples[i].ref.rot.transpose() * b_world[i]);
        }

        // Schur complement: (W^2 I - M M^T) c = W * sum_b - M * sum_AT_b.
        const Eigen::Matrix3d schur = W * W * Eigen::Matrix3d::Identity() - M * M.transpose();
        const Eigen::Vector3d rhs   = W * sum_b - M * sum_AT_b;
        c = schur.colPivHouseholderQr().solve(rhs);
        S = (M.transpose() * c - sum_AT_b) / W;

        // Per-sample residuals: e_i = A_i^T c - S - A_i^T b_world_i.
        std::vector<double> residMag(N);
        double ssq = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            const Eigen::Vector3d e = samples[i].ref.rot.transpose() * c - S
                                      - samples[i].ref.rot.transpose() * b_world[i];
            const double mag = e.norm();
            residMag[i] = mag;
            ssq += mag * mag;
        }
        result.residualRms = std::sqrt(ssq / static_cast<double>(N));

        // Scale estimate.
        double scale = 0.0;
        if (opts.useTukeyBiweight) {
            scale = spacecal::robust::Qn(residMag);
            if (!(scale > kMadFloor)) scale = kMadFloor;
        } else {
            std::vector<double> absResid = residMag;
            std::nth_element(absResid.begin(), absResid.begin() + absResid.size() / 2, absResid.end());
            const double median = absResid[absResid.size() / 2];
            std::vector<double> devs(N);
            for (std::size_t i = 0; i < N; ++i) devs[i] = std::abs(residMag[i] - median);
            std::nth_element(devs.begin(), devs.begin() + devs.size() / 2, devs.end());
            scale = devs[devs.size() / 2];
            if (!(scale > kMadFloor)) scale = kMadFloor;
        }

        const double tuneConstant = opts.useTukeyBiweight
            ? spacecal::robust::kTukeyTune
            : kCauchyTune;
        const double c0 = tuneConstant * scale;

        for (std::size_t i = 0; i < N; ++i) {
            double cThis = c0;
            if (opts.useVelocityAwareWeighting) {
                const double v = std::max(0.0, std::max(samples[i].refSpeed, samples[i].targetSpeed));
                const double vScale = 1.0 + kVelocityKappa * v / kVelocityRefMps;
                cThis = c0 / vScale;
                if (!(cThis > 1e-9)) cThis = 1e-9;
            }
            const double r = residMag[i];
            double w = 0.0;
            if (opts.useTukeyBiweight) {
                w = spacecal::robust::TukeyWeight(r, cThis);
            } else {
                const double rOverC = r / cThis;
                w = 1.0 / (1.0 + rOverC * rOverC);
            }
            weights[i] = w;
        }

        double maxDelta = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            const double denom = std::max(std::abs(prevWeights[i]), 1e-9);
            const double rel = std::abs(weights[i] - prevWeights[i]) / denom;
            if (rel > maxDelta) maxDelta = rel;
        }
        prevWeights = weights;

        result.iterations = iter + 1;
        if (iter > 0 && maxDelta < kWeightChangeThreshold) break;
    }

    // Condition ratio: JacobiSVD on the normalized Schur matrix.
    {
        double W = 0.0;
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < N; ++i) {
            W += weights[i];
            M.noalias() += weights[i] * samples[i].ref.rot;
        }
        if (W > 0.0) {
            const Eigen::Matrix3d schurNorm =
                Eigen::Matrix3d::Identity() - (M / W) * (M / W).transpose();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(schurNorm);
            const auto& sv = svd.singularValues();
            result.conditionRatio = (sv(0) > 0.0) ? std::sqrt(sv(2) / sv(0)) : 0.0;
        }
    }

    result.translation = c;
    result.offset      = S;

    static auto s_lastLog = std::chrono::steady_clock::time_point{};
    const auto nowTp = std::chrono::steady_clock::now();
    if (nowTp - s_lastLog >= std::chrono::seconds(2)) {
        s_lastLog = nowTp;
        char buf[224];
        std::snprintf(buf, sizeof buf,
            "cal_translation_solve_direct: rms=%.6f cond=%.6f iters=%d N=%zu",
            result.residualRms, result.conditionRatio, result.iterations, N);
        Metrics::WriteLogAnnotation(buf);
    }

    return result;
}

DirectResult CalibrateTranslationUpstream(
    const std::vector<Sample>& samples,
    const Eigen::Matrix3d& C_R)
{
    DirectResult result;

    // The pairwise loop pushes two rows per (i, j) pair with j < i, so the
    // final count is 2 * C(N, 2) = N * (N - 1). The previous N * N * 2
    // reservation over-allocated by ~2x.
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;
    const std::size_t N = samples.size();
    deltas.reserve(N > 1 ? N * (N - 1) : 0);

    for (std::size_t i = 0; i < N; i++) {
        Sample s_i = samples[i];
        s_i.target.rot = C_R * s_i.target.rot;
        s_i.target.trans = C_R * s_i.target.trans;

        for (std::size_t j = 0; j < i; j++) {
            Sample s_j = samples[j];
            s_j.target.rot = C_R * s_j.target.rot;
            s_j.target.trans = C_R * s_j.target.trans;

            auto QAi = s_i.ref.rot.transpose();
            auto QAj = s_j.ref.rot.transpose();
            auto dQA = QAj - QAi;
            auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
            deltas.push_back(std::make_pair(CA, dQA));

            auto QBi = s_i.target.rot.transpose();
            auto QBj = s_j.target.rot.transpose();
            auto dQB = QBj - QBi;
            auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
            deltas.push_back(std::make_pair(CB, dQB));
        }
    }

    if (deltas.empty()) {
        return result;
    }

    Eigen::VectorXd constants(deltas.size() * 3);
    Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

    for (std::size_t i = 0; i < deltas.size(); i++) {
        for (int axis = 0; axis < 3; axis++) {
            constants(i * 3 + axis) = deltas[i].first(axis);
            coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
        }
    }

    Eigen::BDCSVD<Eigen::MatrixXd> svd(coefficients,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    result.translation = svd.solve(constants);

    // Emit a throttled diagnostic with the *actual* SVD ratio of the upstream
    // pairwise system. Visible to the user via `cal_translation_cond_upstream`
    // so the upstream-mode motion buffer's observability isn't a black box.
    const auto& sv = svd.singularValues();
    const double actualRatio = (sv.size() > 0 && sv(0) > 0.0)
        ? std::sqrt(sv(sv.size() - 1) / sv(0))
        : 0.0;

    static auto s_lastUpstreamCondLog = std::chrono::steady_clock::time_point{};
    const auto nowTp = std::chrono::steady_clock::now();
    if (nowTp - s_lastUpstreamCondLog >= std::chrono::seconds(2)) {
        s_lastUpstreamCondLog = nowTp;
        char buf[224];
        std::snprintf(buf, sizeof buf,
            "cal_translation_cond_upstream: svd_ratio=%.6f sample_count=%zu",
            actualRatio, N);
        Metrics::WriteLogAnnotation(buf);
    }

    // Set the conditionRatio the dispatcher will write into
    // m_translationConditionRatio to 1.0. The fork-added 0.05 condition gate
    // is bypassed in upstream mode (upstream had no such gate); the actual
    // ratio is emitted to the log above for visibility.
    result.conditionRatio = 1.0;
    result.iterations = 1;
    return result;
}

} // namespace spacecal::translation
