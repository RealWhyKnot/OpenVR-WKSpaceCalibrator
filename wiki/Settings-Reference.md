# Settings Reference

Plain-language explanation of every user-facing toggle in the WhyKnot fork. Everything in **Basic** appears on the main tab; everything in **Advanced** is behind the Advanced tab.

If you're new to the program, the short version is: pick **AUTO** for every option that has it (it's the default), don't touch anything in Advanced, run the [[Setup Wizard]] once. The rest of this page is here for when something looks off and you want to know what knob does what.

---

## Basic

### Jitter threshold

**What it does:** the maximum sample-to-sample noise (translation std-dev, in some abstract unit -- think "a few mm at the default") that the calibration math will tolerate before refusing to start a one-shot calibration. If a tracker is reporting noisier-than-this poses, we abort with "Reference/Target device is not tracking" rather than fitting against unreliable data.

**Default:** 3.0. Set it lower (closer to actual jitter) for stricter quality gating; raise it for systems with noisy IMU trackers.

**When you'd change it:** almost never. The default is loose; the failure mode of going too tight is an aborted one-shot calibration with a misleading "not tracking" message. Adjust only if you're seeing aborts despite trackers that look clean.

### Recalibration threshold

**What it does:** during continuous calibration, controls how much *better* a candidate must be before it replaces the current calibration. Specifically: a new candidate is accepted only when `priorError < newError × threshold` (i.e. the new error must be at least `threshold` times smaller than the prior). Default 1.5.

**Default:** 1.5. The recent math fix also enforces a 5 mm floor on the prior so once you converge to sub-mm errors, the gate stops tightening to absurdity (which used to spuriously trip the watchdog).

**When you'd change it:** raise it (toward 5-10) for very drifty IMU trackers where you want recalibration to be more conservative — the math has to find a *much* better solution to swap in. Lower it (toward 1.1) only if you suspect the math is being too conservative and missing legitimate updates.

### Lock relative position (tristate: Off / On / Auto)

**What it does:** controls whether continuous calibration is allowed to re-solve the *relative* pose between the reference and target devices.

- **Off** — math is free to re-solve every cycle. Right for **independent** devices: HMD on your head, body tracker on your hip, no rigid link between them.
- **On** — once a relative pose has been calibrated, freeze it. Math only updates the world-anchor frame. Right for **rigid attachments**: tracker glued to the headset, taped to a controller during one-shot calibration, etc.
- **Auto** (default) — detect rigidity from observed motion. Starts unlocked; flips to "effectively locked" once the relative pose has stayed stable (within ~5 mm and ~1°) for ~30 consecutive accepted samples (~15 s at the continuous tick cadence). Flips back to unlocked if the variance climbs again, e.g. you repositioned the tracker.

**Why Auto exists:** most users don't know whether their target is rigidly attached to their HMD or not. The failure mode of "miss the rigid attachment" is "calibration slowly chases sensor noise" — not catastrophic, but it shows up as drift. Auto picks the right answer from observation; the resolved status is shown in the Basic tab under the radio so you can see what it decided.

When Auto is selected, the UI shows one of:

- *"Auto: collecting motion data (X/30 samples)"* — not enough observations yet
- *"Auto: locked (detected as rigidly attached, 30 samples)"* — variance was below threshold
- *"Auto: unlocked (devices move independently)"* — variance was above threshold

### Require trigger press to apply

**What it does:** while a controller trigger is held, the calibrated offset is applied to tracked devices; release the trigger and devices snap back to raw poses.

**Default:** off.

**When you'd use it:** sanity-checking a fresh calibration. Hold trigger → "calibrated" world. Release → raw world. If the calibrated world looks correct (your virtual hands match your real hands, etc.) and the raw world looks wrong, you've confirmed the calibration is doing useful work. Release once you're satisfied and turn the toggle off — leaving it on means you have to hold a trigger to be calibrated, which is bad for actual play.

### Recalibrate on movement

**What it does:** when continuous calibration accepts a new offset, the driver only blends toward it *while the device is physically moving* (>5 mm or ~1° per frame). A stationary user (lying down, sitting still) doesn't see calibration shifts — the catch-up happens during their next natural motion, hidden by the actual movement instead of looking like phantom body drift.

**Default:** ON.

**When you'd turn it off:** if you specifically want instantaneous time-based blending regardless of motion. Generally not recommended — the failure mode of "phantom body shift while motionless" is much more annoying than the rare case where you'd want instant correction during sitting still.

### Enable debug logs

**What it does:** writes a per-tick CSV log of calibration state to `%LocalAppDataLow%\SpaceCalibrator\Logs\spacecal_log.<date>.txt`. The **Logs** tab lists captured sessions with quick-actions (Open folder / Copy path / Open file) for attaching them to bug reports.

**Default:** off.

**Cost:** tiny disk I/O per tick; safe to leave on for bug reports. Reachable from the Settings panel in **both** continuous and non-continuous mode (previous versions only exposed the toggle inside the Continuous Calibration window).

---

## Advanced

### Hide tracker

**What it does:** suppresses the target tracker's pose in OpenVR while continuous calibration is running. Other apps see it as "not tracked".

**When you'd use it:** the tracker is a calibration aid only — e.g. you taped a Vive tracker to a Quest controller for one-shot calibration. You don't want the tracker showing up in VRChat as a duplicate of the controller. Turn this on; the tracker becomes invisible to the rest of OpenVR while still feeding the math.

**When you wouldn't:** independent body trackers. You absolutely want them visible — they're full-body inputs.

**Default:** off.

### Static recalibration

**What it does:** when a relative pose has been recorded (Lock relative position is On, or Auto has detected rigidity), and the live solver's estimate diverges noticeably from that locked pose, snap to the locked solution instead of waiting for incremental convergence.

**When you'd use it:** rigid attachments where you want fast recovery from tracking glitches. After a brief tracking loss, the solver might produce a sketchy estimate; static recalibration pulls it back to the known-good locked relative pose instead of incrementally fighting back.

**When you wouldn't:** independent devices. There's no fixed relative pose to snap to; the feature is a no-op (so leaving it on is harmless).

**Default:** **on** (changed in this fork from upstream's off-by-default). It's a no-op when there's nothing locked to snap to, and accelerates recovery on rigid setups -- safer to leave on by default.

### Ignore outliers

**What it does:** drops sample pairs whose rotation axis disagrees with the consensus before the LS solve. Robustifies the math against intermittent USB glitches, brief tracking loss, or a tracker that briefly tracks through occlusion.

**Default:** off (in code; the tooltip historically said "default on" — the tooltip is wrong, the field initialises false). Most users probably want it ON.

**When you'd disable:** if you suspect the outlier rejector is throwing out genuinely-good samples. Rare. Most setups want this on.

### Calibration speed: Auto / Fast / Slow / Very Slow

**What it does:** sets the size of the rolling sample buffer the math uses. More samples = more averaging, smoother result, slower convergence. Fewer samples = faster response, more noise.

- **Auto** (default) — picks the right speed from observed jitter on both reference and target trackers. Sub-mm jitter → Fast; 1-5 mm → Slow; >5 mm → Very Slow. Sticky with hysteresis so it doesn't oscillate.
- **Fast** — 100-sample buffer. Fastest convergence; right for clean lighthouse setups.
- **Slow** — 250-sample buffer. Smoother; right for typical mixed setups.
- **Very Slow** — 500-sample buffer. Maximum smoothing; right for noisy / reflective rooms or drift-prone IMU trackers.

When AUTO is on, the Advanced tab shows the resolved bucket and the observed jitter values that drove the decision.

### Latency / tracker offset / playspace scale

These three are deeper-tuning knobs covered in [[Continuous Calibration]] § Inter-system latency offset. Usually you don't touch them; the math has auto-detection for the latency offset. Manual override is there for edge cases (specific wireless trackers that need a static offset).

---

## Other UI surfaces

### Prediction tab

Per-tracker 0-100 smoothness sliders. See [[Prediction Suppression]] for the full story. Three devices are always locked to 0 regardless of what you pick: the HMD, the calibration reference tracker, and the calibration target tracker (suppressing them would corrupt either your view or the math).

### Logs tab

Always visible (replaces the older Recordings tab). Lists every CSV log file the overlay has captured under `%LocalAppDataLow%\SpaceCalibrator\Logs\`, with newest first. Per-row actions:

- **Open selected** -- opens the file in your default text editor.
- **Copy path** -- puts the absolute path on the clipboard, ready for "drag this into a bug report".
- **Open folder** (top-level button) -- pops the Logs directory in Explorer.

A status indicator shows whether logging is currently on (green dot) or off (orange). Toggling debug logs is in the Settings panel, not here -- the tab is intentionally listing-only so you can browse old logs even when you've turned recording off.

The previous "load and replay this log against the live math" flow lived in the Recordings tab; that was dev-tooling rather than user-facing, and now lives only in the standalone replay CLI under `tools/replay/`.

### Mode pill

The coloured `[LIVE]` / `[STANDBY]` / `[FIXED OFFSET ACTIVE]` / `[NO PROFILE]` pill at the top of every tab summarises the calibration state machine in one glance. Hover for an explanation.

### Watchdog / HMD-stall counters

Diagnostic counters under the mode pill. The stuck-loop watchdog clears the calibration if it can't converge for ~25 s; the HMD-stall watchdog purges the sample buffer when the headset stops reporting fresh poses for ~1.5 s. Hover the rows for what they mean. Frequent firings indicate tracking-environment problems (bad lighting, USB bandwidth, etc.).
