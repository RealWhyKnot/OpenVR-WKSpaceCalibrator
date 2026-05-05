# Troubleshooting

Symptoms first, then what's likely going on, then what to try.

## Trackers in the wrong place after I put my HMD back on

**Symptom:** body trackers were fine before you took the headset off
for a sip of water. Putting it back on, they're in the wrong spot.

**What happened:** when a Quest HMD comes out of standby, its inside-out
SLAM can re-localize the room origin slightly. The body trackers
(Lighthouse / SlimeVR / etc.) didn't shift, so the calibrated transform
is now slightly off relative to the new HMD frame.

**What to do:** nothing. Continuous calibration walks the offset back
to truth as you move. The motion-gate floor (10/50/90 percent based on
correction size) keeps things drifting toward the right place even
while you stand still.

If the HMD jump was big enough (30+ cm), the auto-recovery detector
fires automatically: it wipes the calibration and continuous-cal
re-acquires from scratch over ~30 seconds. You'll see your trackers
briefly walk to weird positions, then back to correct. Look for
`auto_recover_from_relocalization` in the spacecal log to confirm
this fired.

See [[Auto Recovery]] for what triggers and what to expect.

## Trackers floating off after a long session

**Symptom:** tracking was fine for hours, now your trackers are
visibly off (5+ cm).

**What happened:** continuous calibration hunts between local minima
when given long-duration unchanging poses (e.g. you sat still in a
movie for 30 minutes). The math drifts within the noise floor.

**What to do:**
1. Move around for ~30 seconds. Continuous-cal has fresh varied poses
   to fit, the offset re-converges.
2. If that doesn't help: check that continuous calibration is actually
   on (top action bar, "Continuous Calibration" button should show
   "Cancel Continuous Calibration"). One-shot calibration doesn't
   self-correct; it's frozen at whatever was solved at the moment
   you ran it.
3. Last resort: open Settings tab, click "Run setup wizard". This
   restarts the whole calibration from a clean slate. You need
   working tracking to do it (the wizard reads poses), so do this
   while seated with your HMD on and a controller in hand.

There's no "Recalibrate now" rescue button mid-session by design:
when calibration is broken, your controllers are exactly the tool
that's broken, so any UI that requires aiming them at a button is
a trap. Auto-recovery for the catastrophic case + natural-motion
convergence for the slow case + Run-setup-wizard for the deliberate
restart covers the design space without that trap.

## Calibration won't complete in the wizard

**Symptom:** the wave step shows "Waiting for first valid
calibration..." and never advances.

Likely causes:

- **Insufficient motion variety.** The math needs rotation in at
  least two axes. Pure-translation or single-axis rotation gives a
  degenerate solve. Wave with yaw + pitch (or yaw + roll), not just
  side-to-side. The Logs tab's Axis Variance plot tells you which
  dimension you're short on.
- **Bad tracking on either device.** If `result != Running_OK` on
  either the reference or the target (occlusion, IMU not settled),
  the math rejects samples. Quick check: does the device move
  smoothly in SteamVR's rendered view? If it stutters, calibration
  won't either.
- **Jitter too high.** The Reference Jitter and Target Jitter plots
  show per-axis std-dev. Above the jitter threshold (default 3 mm),
  calibration won't start. Common cause: a base station that's
  loose, a SlimeVR IMU that hasn't finished thermal-settling, or a
  tracker too close to a base station and getting interference.

## Driver doesn't load after install

**Symptom:** SteamVR starts, the overlay starts, but no offsets
are ever applied.

- Open SteamVR's web console at `http://localhost:8998` (while
  SteamVR is running) and check the loaded-drivers list for
  `driver_01spacecalibrator`. If it's missing, SteamVR didn't find
  or didn't accept the DLL.
- Verify the layout:
  `<drivers>/driver_01spacecalibrator/bin/win64/driver_01spacecalibrator.dll`
  +
  `<drivers>/driver_01spacecalibrator/driver.vrdrivermanifest`.
- Check `<Steam>/logs/vrserver.txt` for load errors. A missing
  dependency (wrong VS runtime) shows up here.

## "Incorrect driver version installed"

**Symptom:** the overlay throws this on startup.

The overlay and driver were built from different commits and their
IPC protocol versions don't match. Reinstall both -- the release zip
ships them together. Extract the latest zip into your SteamVR
`drivers/` directory and re-run the overlay from the same zip.

## Finger smoothing doesn't seem to do anything

**Symptom:** you enabled finger smoothing in the Fingers tab but
your Knuckles fingers still jitter.

- Confirm the driver actually installed the public-vtable hook.
  Grep `<Logs>/driver_log.<timestamp>.txt` for `installed PUBLIC
  IVRDriverInput hooks: vtable[5]=Create, vtable[6]=Update`. If
  this line is missing, the hook didn't take. See [[Finger Smoothing]]
  section "How to verify".
- Confirm strength is non-zero. The Fingers tab has a 0..100 slider.
  At 0, it's passthrough.
- Confirm SteamVR has been restarted since the install. The driver
  loads at SteamVR startup; mid-run installs require a Steam
  shutdown to take effect.

## Where to look for logs

- **Overlay in-app log.** Bottom of the overlay window. Cleared
  when calibration starts; shows recent state-machine transitions
  and watchdog firings.
- **CSV metric log.** `%LOCALAPPDATA%\Low\SpaceCalibrator\Logs\spacecal_log.<timestamp>.txt`.
  Enable via the Logs tab. One row per tick. Rotates per overlay
  session. Logs older than 24 hours are auto-deleted.
- **Driver log.** Same directory, `driver_log.<timestamp>.txt`.
  Driver-side hook installation, IPC handshakes, finger-smoothing
  state.
- **SteamVR vrserver log.** `<Steam>/logs/vrserver.txt`. Driver
  load errors, OpenVR-side issues.
