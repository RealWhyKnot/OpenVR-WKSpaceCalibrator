#pragma once

// Right-invariant Kalman filter on the published calibration transform.
// Replaces the single-step EMA blend (alpha = 0.3) at the publish point in
// ComputeIncremental with a proper predict+update filter that tracks (yaw,
// tx, ty, tz) over time with explicit process and measurement noise models.
//
// Default-off; the EMA path stays as the published default until real-
// session data shows the filter is stable on this hardware.
//
// State (4-D scalar): the calibration is yaw-only (rotation is constrained
// to the world Y axis) plus 3-D translation, so we filter the four scalars
// directly. The "right-invariant" framing in the project memo is overkill
// for a yaw-only rotation -- there is no Lie-group bookkeeping when the
// rotation degree is one. Keeping the state as plain scalars also keeps the
// implementation trivially testable.
//
// Process noise: random walk per second, tuned to typical Quest-SLAM long-
// term drift figures (sigma_yaw ~ 0.1 deg/min, sigma_pos ~ 1 mm/min).
// Measurement noise: fixed defaults representing "the candidate cal passed
// validation gates" (sigma_yaw_meas = 0.5 deg, sigma_pos_meas = 5 mm). Both
// can be tuned per call.
//
// Divergence detection: hard caps on per-component innovation. When a
// candidate is way outside what the filter expected (e.g. post-relocalize
// snap), the caller is responsible for handling -- the helper here just
// flags the divergence so the caller can reset state + take a graceful
// fallback path.
//
// Pure helpers, header-only, testable in isolation.

#include <cmath>

namespace spacecal::blendfilter {

// Process-noise standard deviations per second. Documented in the memo;
// values from typical Quest-SLAM long-term drift behavior. Square these
// and multiply by dt (seconds) to get the variance increment per step.
constexpr double kSigmaYawPerSec = 2.91e-5;     // 0.1 deg/min in rad/s
constexpr double kSigmaPosPerSec = 1.67e-5;     // 1 mm/min in m/s

// Measurement-noise standard deviations per accept. Conservative defaults
// representing "the candidate passed validation"; not tied to per-tick
// solve quality (could be made adaptive in a future revision).
constexpr double kSigmaYawMeas = 0.5 * 3.14159265358979323846 / 180.0;  // 0.5 deg
constexpr double kSigmaPosMeas = 5e-3;                                   // 5 mm

// Divergence thresholds on per-component innovation. Larger than the
// noise budget the filter would otherwise expect; if a candidate is more
// than this away from the predicted state, treat as a teleport (post-
// relocalize snap, geometry-shift recovery, etc.) and let the caller
// reset.
constexpr double kDivergenceYawRad = 0.5;        // ~28.6 deg
constexpr double kDivergencePosM   = 0.30;       // 30 cm

// Initial covariance applied on first accept and after a divergence reset.
// Loose enough that the first measurement dominates, then tightens as the
// filter accumulates updates.
constexpr double kInitialPyaw = 0.1;             // (rad)^2; ~18 deg 1-sigma
constexpr double kInitialPpos = 0.04;            // (m)^2; 20 cm 1-sigma

struct State {
    bool initialized = false;
    // State estimate.
    double yaw = 0.0;
    double tx = 0.0, ty = 0.0, tz = 0.0;
    // Diagonal covariance.
    double Pyaw = kInitialPyaw;
    double Px = kInitialPpos, Py = kInitialPpos, Pz = kInitialPpos;
};

// One-step predict + update against a candidate measurement. Returns the
// component-wise innovation magnitudes via outYawInnov / outPosInnov so the
// caller can log or react to large jumps. Consumes dt (seconds since the
// previous update) for the predict step. On first call (state.initialized
// = false), seeds the state directly to the measurement and returns zero
// innovations.
//
// processSigmaYaw / processSigmaPos default to the long-term drift figures
// above; measSigmaYaw / measSigmaPos to the validation-gate values. Caller
// can override per-tick if it has a better noise estimate (e.g. from the
// solve's condition ratios).
inline void Update(State& state,
                   double measYaw, double measTx, double measTy, double measTz,
                   double dt,
                   double& outYawInnov, double& outPosInnov,
                   double processSigmaYaw = kSigmaYawPerSec,
                   double processSigmaPos = kSigmaPosPerSec,
                   double measSigmaYaw    = kSigmaYawMeas,
                   double measSigmaPos    = kSigmaPosMeas) {
    if (!state.initialized) {
        state.yaw = measYaw;
        state.tx = measTx; state.ty = measTy; state.tz = measTz;
        state.Pyaw = kInitialPyaw;
        state.Px = kInitialPpos; state.Py = kInitialPpos; state.Pz = kInitialPpos;
        state.initialized = true;
        outYawInnov = 0.0;
        outPosInnov = 0.0;
        return;
    }

    // Predict: x stays (random-walk model), P grows by Q * dt.
    if (dt < 0.0) dt = 0.0;
    const double dQyaw = processSigmaYaw * processSigmaYaw * dt;
    const double dQpos = processSigmaPos * processSigmaPos * dt;
    const double Pyaw_pred = state.Pyaw + dQyaw;
    const double Px_pred   = state.Px   + dQpos;
    const double Py_pred   = state.Py   + dQpos;
    const double Pz_pred   = state.Pz   + dQpos;

    // Innovation: measurement minus predicted state. Wrap yaw to [-pi, pi]
    // so a measurement near +pi and a state near -pi give a small delta,
    // not a 2*pi jump.
    const double yYawRaw = measYaw - state.yaw;
    const double yYaw = std::atan2(std::sin(yYawRaw), std::cos(yYawRaw));
    const double yX   = measTx  - state.tx;
    const double yY   = measTy  - state.ty;
    const double yZ   = measTz  - state.tz;
    outYawInnov = std::abs(yYaw);
    outPosInnov = std::sqrt(yX * yX + yY * yY + yZ * yZ);

    // Update with H = I, R diagonal: K = P / (P + R) component-wise.
    const double Ryaw = measSigmaYaw * measSigmaYaw;
    const double Rpos = measSigmaPos * measSigmaPos;
    const double Kyaw = Pyaw_pred / (Pyaw_pred + Ryaw);
    const double Kx   = Px_pred   / (Px_pred   + Rpos);
    const double Ky   = Py_pred   / (Py_pred   + Rpos);
    const double Kz   = Pz_pred   / (Pz_pred   + Rpos);

    state.yaw += Kyaw * yYaw;
    state.tx  += Kx   * yX;
    state.ty  += Ky   * yY;
    state.tz  += Kz   * yZ;

    state.Pyaw = (1.0 - Kyaw) * Pyaw_pred;
    state.Px   = (1.0 - Kx)   * Px_pred;
    state.Py   = (1.0 - Ky)   * Py_pred;
    state.Pz   = (1.0 - Kz)   * Pz_pred;
}

// Per-component divergence test against the innovations from the most
// recent Update call. True iff EITHER yaw or position innovation exceeded
// the divergence cap. Caller is expected to react by resetting state to
// the measurement and falling through to a non-filtered blend for the
// current tick (e.g. the existing EMA path).
constexpr bool IsDivergent(double yawInnov, double posInnov,
                            double divYaw = kDivergenceYawRad,
                            double divPos = kDivergencePosM) {
    return yawInnov > divYaw || posInnov > divPos;
}

// Reset the filter so the next Update reseeds from a fresh measurement.
inline void Reset(State& state) {
    state.initialized = false;
    state.yaw = 0.0;
    state.tx = 0.0; state.ty = 0.0; state.tz = 0.0;
    state.Pyaw = kInitialPyaw;
    state.Px = kInitialPpos; state.Py = kInitialPpos; state.Pz = kInitialPpos;
}

} // namespace spacecal::blendfilter
