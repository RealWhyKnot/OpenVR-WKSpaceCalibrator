# Changelog

All notable changes to OpenVR-SpaceCalibrator (RealWhyKnot fork). The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses date-driven versioning (`YYYY.M.D.N` for releases, `YYYY.M.D.N-XXXX` for dev builds -- see [Building](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Building) for shape rules).

The most recent release is at the top.

<!-- Entries under "## Unreleased" are appended automatically by the changelog-append GitHub
     workflow on every push to main, then promoted to the versioned section by release.yml when
     a tag is cut. Don't hand-edit Unreleased -- your edits will be overwritten on the next push.
     To override an entry, amend the commit subject before the push. -->

## Unreleased

_No notable changes since the last release._

---

## [v2026.5.5.0](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/releases/tag/v2026.5.5.0) -- 2026-05-06

### Added
- **diagnostics:** "who moved" annotation on relocalization fire (a2e638d)
- **experimental:** Kalman-filter blend at publish toggle (c1d61fe)
- **experimental:** Tukey biweight + Qn-scale IRLS kernel toggle (af4a3af)
- **experimental:** Velocity-aware outlier weighting toggle (99e6638)
- **experimental:** CUSUM geometry-shift detector toggle (7da178e)
- **ui:** Surface latency auto-detect + GCC-PHAT toggles in advanced panel (9b009f4)
- **diagnostics:** GCC-PHAT latency option + sustained-tilt annotation (c2fdef7)
- **blend:** Smart-by-correction-size motion-gate floor + auto-recovery snap (a7c19aa)
- **driver:** Hook PUBLIC IVRDriverInput vtable[5/6] for finger smoothing (f812002)
- **overlay:** Launch-context diagnostics for script-vs-Start-menu diff (33d3bf0)
- **driver+overlay:** Finger smoothing for Index Knuckles (f8fcb75)

### Changed
- Cpu_pressure_warning + cpu_pressure_spike annotations (2147535)
- Profile_loaded / profile_saved annotations (aa388f0)
- Session config dump + toggle-flip annotations (6706bb7)
- Log prior cal state on recovery (5e53131)
- **ui:** Split Experimental toggles into their own panel (1f7b4c4)
- **driver:** Remove IVRDriverInputInternal hook chase + thunk parser + DeepProbe (67e49af)
- Forensic log lines for audit P2 rows upstream-issue-6, upstream-issue-8, upstream-issue-11, upstream-issue-12 (44674ac)
- **overlay:** Comprehensive debug logging for math + auto-recovery + profile-load (4c4e7b9)
- ux(overlay): add 'Recalibrate from scratch' button to Basic tab Actions (09e55eb)
- **driver:** Replace finger-config mutex with atomic<uint64_t> (786073a)
- fix+ux(overlay): corroborated auto-recovery + sticky banner with Undo (76b6755)
- feat+fix(overlay): auto-recovery from Quest re-localization (hardened) (8b0f116)
- feat+fix(calibration): two-phase one-shot, watchdog wedge, pose validity, diagnostics (468cd68)

### Fixed
- **experimental:** Cap Qn input + correct profile-cm unit (2f0a059)
- **calibration:** Remove wedge guard -- caused reset loop on user setup (f785935)
- **driver:** Three P2 cleanups from the post-deploy audit (1819e8b)
- **config:** Guard ReadRegistryKey against size==0 underflow (89084d3)
- **calibration:** Silent auto-recover from wedged cals + revert HMD-stall buffer purge (cb1dedf)
- **overlay:** Revert per-base auto-recovery corroboration; restore 10s post-stall grace (310580d)
- **calibration:** Exact yaw projection via swing-twist + true SVD condition gate (7ac3c52)
- **driver:** Write log to %LocalAppDataLow% next to overlay's, not vrserver cwd (92ffc87)
- **driver:** Close g_driver race + filter DeepProbeInterface garbage vtables (15f659f)
- **driver:** Drain in-flight detours before Cleanup tears down state (2726a87)
- **driver:** Adaptive-rate threshold, quaternion drift, HMD-stall mutex contention (0b3df9f)

---
