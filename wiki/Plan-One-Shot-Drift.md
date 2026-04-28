# Plan: Helping one-shot calibration users with drift

Working draft. Not implemented yet — this is a proposal to react to before any code.

## Who this is for

Users who run one-shot calibration once at the start of a session and don't want continuous calibration eating cycles in the background. They may have:

- A standalone HMD (Quest, Pico, Bigscreen Beyond) whose inside-out tracking origin drifts a few cm per hour.
- A "set and forget" attitude — they want to calibrate once, play, log off, and not think about it.
- Concerns about continuous calibration changing the offset mid-session in ways they can feel (even with recalibrate-on-movement).

For lighthouse-only setups one-shot is fine indefinitely (lighthouse origins don't drift within a session). The interesting case is mixed setups where the *HMD's* origin drifts but the secondary tracking system's origin is stable.

## What already helps one-shot users (this fork, no extra work)

The following improvements landed this session and apply equally to one-shot users:

- **SO(3) Kabsch + Rodrigues yaw projection** in `ComputeOneshot`. Cleaner yaw recovery when the two systems' gravity axes don't perfectly agree.
- **IRLS with Cauchy weighting** on the translation solve. Robust against heavy-tailed jitter (Slime IMU translations, USB-glitched frames).
- **Condition-ratio guards** in both rotation and translation solves. Refuses to produce a result when motion is too planar / single-axis. The user's Calibrate button gives an explicit "rotate around more axes" message instead of silently fitting against degenerate data.
- **Welford jitter fix.** The pre-calibration jitter check now reports real values (was inflated ~2x). The `Jitter threshold` slider in Basic now means what it says.
- **Per-tracker prediction smoothness slider.** Cleaner pose data feeding the math regardless of mode.
- **Per-tracking-system fallback transforms.** Even in one-shot mode, a tracker that connects after calibration inherits the current offset on its first pose update — same as continuous mode. So adding a body tracker mid-session "just works" in one-shot too.

What one-shot users **don't** get (because it's by definition continuous):
- Watchdog escapes from stuck math
- Auto-lock detection of rigid attachment
- Sub-mm convergence over time
- Drift correction as the HMD's tracking origin shifts

That last one is the gap to close.

## The drift problem

```
t=0    HMD origin: [0, 0, 0]      Tracker offset = (12cm, 0, 0)   World looks correct.
t=2h   HMD origin: [3cm, 0, -2cm]  Tracker offset still (12cm, 0, 0).
                                    Tracker now appears 3cm too far in X
                                    and 2cm too far back in Z.
```

The calibration is still mathematically valid w.r.t. the HMD's *current* coordinate frame, but the HMD's frame has drifted relative to the room. The body tracker, anchored to the calibration-time HMD origin, looks displaced.

For continuous mode, this is mostly invisible: the math keeps re-solving as the HMD drifts, and `recalibrateOnMovement` hides each correction in the user's natural movement. For one-shot mode, drift accumulates until the user notices and recalibrates.

## Five options, ranked by effort

### Option A: Drift watchdog (RECOMMENDED)

**What:** Passive background sampler that continues running after one-shot calibration finishes. It doesn't *update* the calibration — it just measures how well the existing offset still fits the current pose stream. When residuals climb past a threshold, surface a user-visible badge: "Calibration drift detected — click here to recalibrate".

**Mechanics:** every ~1 s, take a fresh ref+target pair and compute the residual `||target_world - calibratedTransform(target) - ref_world||`. EMA over a 30-s window. Badge fires when EMA exceeds say 3 cm (configurable) for sustained ~30 s.

**User experience:** unchanged in the typical case (no badge → user keeps playing). Drift becomes self-announcing. One-click recalibrate kicks off a fresh ~5-second motion capture, replaces the offset.

**Effort:** ~1 day. The sampling logic exists already (it's what `CollectSample` does). New piece is the residual-monitor + badge.

**Risk:** false positives (user moved a body tracker manually → looks like drift → badge). Mitigate by also tracking tracker velocity — if the tracker just moved a lot recently, suppress the alert.

### Option B: Real-time motion-quality feedback during one-shot

**What:** while the user is doing the figure-8 wave for one-shot calibration, show a live progress bar that fills based on motion *quality*, not just sample count. "Rotation diversity: 60%". "Translation diversity: 90%". The "Calibrate" finishes automatically when both cross a threshold; if the user stops moving with poor coverage, show specific feedback ("Try rotating around the up axis too").

**Mechanics:** instrument `DetectOutliers` to expose its per-axis variance breakdown to the UI. Map it to two progress bars. Auto-trigger `ComputeOneshot` when both bars are full.

**User experience:** much higher chance the resulting calibration is good. New users currently do a ~3 second random wave and get sub-optimal results; this guides them.

**Effort:** ~1 day. The math exposes most of what's needed already (`m_rotationConditionRatio`, `m_translationConditionRatio`, `m_axisVariance`).

**Risk:** low. UI guidance, not math change.

### Option C: Stationary-anchor mode

**What:** offer a calibration mode where the user designates a tracker as a "world anchor" — typically taped to the floor or wall, not worn. The math then runs with the anchor as the reference (instead of the HMD). The HMD becomes a regular tracked device whose pose is re-aligned to the anchor.

**Why this helps:** the anchor tracker doesn't drift (it's stationary, in a system that doesn't drift like lighthouse). The HMD's drift becomes a measurable per-frame correction applied to *everything else*. Body trackers stay aligned because they share the anchor's space.

**Mechanics:** the existing math already supports this — you just pick the anchor tracker as the reference instead of the HMD. The wizard would gain a "Do you have a fixed reference tracker (taped to the wall/floor)?" question.

**Effort:** ~3 days. UI changes (wizard branch, reference picker), plus more careful handling of the case where ref isn't the HMD (some code paths assume ref = HMD).

**Risk:** medium. The math is fine, but a bunch of code (auto-lock detector, multi-ecosystem fallback dispatch) currently special-cases the HMD as reference. Each of those needs review.

### Option D: HMD recenter event integration

**What:** subscribe to OpenVR's `VREvent_TrackedDeviceUserInteractionEnded` / `VREvent_SeatedZeroPoseReset` events. When the user explicitly recenters their HMD, immediately apply a compensating transform to the saved offset.

**Why:** explicit recenter is the most common form of HMD origin shift. If the user long-presses the Home button on a Quest, the entire HMD coordinate frame translates/rotates — this is detectable via OpenVR events.

**Mechanics:** listen to the events, compute the delta between pre-recenter and post-recenter HMD pose, apply the inverse to the calibrated transform.

**Effort:** ~half a day. Mostly event plumbing.

**Risk:** low. Doesn't help with gradual drift, only explicit recenter — but explicit recenter is the failure mode that produces the most visible jumps, so handling it cleanly is worthwhile.

### Option E: Automatic micro-recalibration without UI

**What:** like Option A but instead of surfacing a badge, transparently run a brief continuous-style recalibration when drift is detected. User never sees it.

**Mechanics:** drift watchdog from Option A, but instead of badging, kick off `ComputeIncremental` for ~10 s, accept if RMS < threshold, otherwise fall back to badge.

**User experience:** "it just works" — never click anything.

**Effort:** ~2 days (Option A + adaptive trigger).

**Risk:** medium. The classic continuous-calibration failure modes (degenerate motion → bad solution) all apply, just hidden from the user. If the auto-recalibration produces a worse offset than the original, the user sees jumpy trackers with no clear explanation. Need conservative acceptance criteria.

## My recommendation

Build **A** + **B** as one feature set:

- **B** improves the *initial* one-shot result, so drift starts from a better anchor.
- **A** catches drift as it accumulates and gives the user a clean recovery path.

Together they cover the lifecycle — start strong, recover when you drift. Total effort: ~2 days.

**D** is a small bonus that pairs well with A (recenter events become "drift detected, here's a one-click reset").

**C** and **E** are bigger investments. Defer until A+B+D have been used in the wild and we see whether the gap is still painful enough to justify them.

## Open questions

- **What's the right drift threshold?** 3 cm is a guess. Should be configurable, with a sensible default. We'd want telemetry from real users to dial it in.
- **Does the badge appear in VR or in the desktop UI?** Both — but the in-VR badge would need to be subtle (small overlay corner) so it doesn't break immersion.
- **Should we track drift per-system (multi-ecosystem)?** Yes — each non-HMD system has its own calibration, each can drift independently against its HMD reference. The watchdog should run per-system.
- **What's the failure mode if the user is *intentionally* moving a body tracker (e.g. carrying it from one foot to another)?** The watchdog would falsely flag drift. Velocity-suppression handles short bursts; we may need a longer "ignore drift while heavy motion is happening" rule.

## Next step if approved

1. Land **B** first — it's UI-only and risk-free.
2. Land **A** second — backend math + a single banner UI element.
3. Add **D** as a small follow-up.
4. Re-evaluate whether C / E are still worth doing once A + B are in real users' hands.
