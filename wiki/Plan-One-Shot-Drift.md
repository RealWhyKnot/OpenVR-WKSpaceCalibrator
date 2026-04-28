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

## Options, ranked by effort

### Option A: Drift watchdog (RECOMMENDED, foundational)

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

---

## Exploit-user-habits ideas

The previous five options treat the user as someone who'll click a button when prompted. The next group exploits things users *already do* (often for entirely unrelated reasons) as natural calibration anchors. The big one is T-pose detection, which dovetails with the social-VR T-pose-for-IK ritual.

### Option F: T-pose detection as a drift anchor (HIGHEST-VALUE auto-trigger)

**The insight:** VRChat (and most social VR with IK) asks users to T-pose to calibrate their avatar. Users do this every session, often multiple times per session. T-pose is:

- A well-defined geometric configuration: HMD upright, arms extended laterally at roughly shoulder height, body still
- Held for 1-3 seconds (long enough to detect with confidence)
- Repeated naturally (no behavior change required)
- Independent of body shape *for relative geometry* (the constellation of trackers in T-pose is consistent for the same user across sessions, even if absolute dimensions vary)

**What we'd do:**

1. **Detect T-pose** — heuristic geometry classifier running every tick. Criteria, all required for ~1-2 sustained seconds:
   - HMD pitch within ±15° of upright (looking roughly forward)
   - Hand controllers (if present) at HMD height ±30 cm, lateral distance from HMD > ~50 cm both sides, palms roughly parallel to floor
   - Body velocity < 5 cm/s on every tracked device (user is still)
   - Hip tracker (if present) below HMD by ~50-60 cm (typical waist drop), not rotated significantly relative to HMD yaw

   These are loose enough to handle different user heights / arm spans, tight enough that incidental hand positions don't trigger.

2. **Snapshot the constellation** — on the first detected T-pose post-calibration, record the relative pose between every tracker pair and the HMD. This is the "drift baseline".

3. **Compare on subsequent T-poses** — when T-pose is detected again, compute the relative-pose delta vs the baseline. Translation deltas above some threshold (e.g. 3 cm) indicate drift; rotation deltas above some threshold (~3°) indicate yaw drift.

4. **Surface the drift** — same badge mechanism as Option A. "Drift detected during T-pose check — recalibrate?". User can ignore (badge stays until next clean T-pose) or accept (kick off a brief recalibration).

**Why this works better than passive drift watching (Option A) for the one-shot crowd:**

- **Higher signal-to-noise.** Comparing T-pose to T-pose is comparing two stable, well-defined moments. Comparing arbitrary motion samples (Option A) has more noise — sometimes the residual is high just because the user is moving fast.
- **Cleaner UX trigger.** Users naturally land in T-pose for unrelated reasons; using that as the check point feels invisible. Option A's residual-EMA would fire at unpredictable times.
- **Free continuous re-validation.** Every avatar setup is a calibration sanity check. A user who T-poses 5 times in a session gets 5 free drift checks.

**Effort:** ~2 days. T-pose detector + snapshot/compare logic + badge integration.

**Risk:** medium. The detector is heuristic — false positives (user happened to be in a T-pose-like configuration) and false negatives (user is too short / has unusual arm length / not in VRChat) both possible. Mitigate with conservative thresholds.

**Cool extension:** T-pose-triggered automatic mini-calibration. Once we know the user is in a known stable pose, we can run `ComputeOneshot` against the recent ~30 s of motion samples (which they generated arriving at the T-pose) without the user explicitly invoking calibration. Happens silently between avatar setups in VRChat.

### Option G: Floor-touch / known-height anchors

**What:** tracked device whose Y coordinate drops below ~5 cm and stays there for ~1 s is "on the floor". That gives us a known absolute anchor (height = 0). If the calibrated offset says the tracker is at Y=15 cm but the device-reported Y is at 0 cm, that's drift.

**Why:** users put trackers on the floor naturally — set them down between play sessions, knock them off their belt onto the floor, etc. Each floor-touch is an anchor moment.

**Effort:** ~1 day. Detector + Y-residual check.

**Risk:** low. Floor detection is unambiguous when the device is genuinely stationary on a flat surface. Filter out fast-moving "floor pass-throughs" (foot stepping on floor briefly during a step) by requiring stillness.

### Option H: Idle-pose averaging

**What:** when all trackers have been still for ~5 s, sample multiple frames and average them into a single high-confidence "stationary moment" sample. Use those samples (instead of motion samples) to anchor a brief recalibration.

**Why:** stationary samples have the lowest jitter. Averaged, they give a cleaner ground truth than any single motion-sample pair.

**Effort:** ~1 day.

**Risk:** low. Worst case is no idle moment is detected during a session and the feature does nothing.

### Option I: Walking / step detection

**What:** detect step cadence from hip-tracker Y-axis oscillation. Each step gives us a known geometric pattern (left-right mirror, forward gait). Use the symmetry as a drift anchor.

**Why:** users walk constantly in VR. Free recalibration anchor.

**Effort:** ~3 days. Step detection is well-studied but tuning it for the variety of VR setups is non-trivial.

**Risk:** medium. False detections from non-walking motion (squatting, leaning).

### Option J: Steam dashboard / boundary snap

**What:** when the SteamVR dashboard opens, the user's HMD and controllers are typically in a canonical "looking at menu" pose. Use this as an anchor.

**Why:** dashboard-open is a frequent event, easy to detect.

**Effort:** ~half a day.

**Risk:** medium. The user's hand position when summoning the dashboard varies a lot; not as repeatable as T-pose.

---

## Updated recommendation

**Phase 1 (next): A + B + F.**

- **B** improves the *initial* one-shot calibration quality.
- **A** catches gradual drift via passive residual monitoring.
- **F** catches drift at high-confidence anchor moments (T-pose) — the killer feature for VRChat users.

A and F complement each other: A is always-on but noisy; F is event-driven and clean. Together, drift is detected reliably without being annoying. Total effort: ~4 days.

**Phase 2 (later): D + G + H.**

- **D** handles explicit recenters (small, cheap, complements F nicely).
- **G** floor-touch anchors are easy and add another natural calibration moment.
- **H** idle-pose averaging gives cleaner samples for any of the above.

**Defer: C, E, I, J.**

- **C** stationary-anchor mode is genuinely useful but invasive (HMD-as-ref assumed in many places). Re-evaluate after seeing how A/B/F+D land.
- **E** silent auto-recalibration is risky enough that explicit user confirmation (A's badge) is probably the better default.
- **I** walking detection is high effort, complex tuning.
- **J** dashboard pose has too much variance to be a reliable anchor.

## Open questions

- **What's the right drift threshold?** 3 cm is a guess. Should be configurable, with a sensible default. We'd want telemetry from real users to dial it in.
- **Does the badge appear in VR or in the desktop UI?** Both — but the in-VR badge would need to be subtle (small overlay corner) so it doesn't break immersion.
- **Should we track drift per-system (multi-ecosystem)?** Yes — each non-HMD system has its own calibration, each can drift independently against its HMD reference. The watchdog should run per-system.
- **What's the failure mode if the user is *intentionally* moving a body tracker (e.g. carrying it from one foot to another)?** The watchdog would falsely flag drift. Velocity-suppression handles short bursts; we may need a longer "ignore drift while heavy motion is happening" rule.
- **Does T-pose detection need to know the user is in VRChat specifically?** No — it's a geometric check, not an app-aware one. Will fire any time the user is in a T-pose, regardless of which game is running. That's a feature: works in VRChat, ChilloutVR, Resonite, NeosVR, anywhere with IK calibration.
- **Should T-pose snapshot persist across sessions?** Tempting, but no — body proportions appear to vary across sessions for the same user (different shoes, different tracker positions). Snapshot per session is safer.

## Next step if approved

1. Land **B** first — it's UI-only and risk-free.
2. Land **A** second — backend math + a single banner UI element.
3. Land **F** third — the highest-value auto-trigger. Builds on A's badge mechanism.
4. Add **D** as a small follow-up (explicit recenter handling).
5. Re-evaluate G / H / and the deferred items based on real-world feedback.
