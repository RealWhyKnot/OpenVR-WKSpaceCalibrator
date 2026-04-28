# spacecal_replay

Offline replay harness for SpaceCalibrator CSV captures.

## What it does

Reads a `spacecal_log_v2` CSV produced by the overlay (Debug tab → "Enable
metrics log") and re-feeds the captured per-tick reference + target poses into a
fresh `CalibrationCalc` instance. Lets you ask questions like *"what would the
new outlier-rejection threshold do to this user's data?"* without needing the
user's hardware.

The captured logs live at:

```
%LOCALAPPDATA%\Low\SpaceCalibrator\Logs\spacecal_log.<timestamp>.txt
```

## Building

The tool is a CMake target inside the main build. From the repo root:

```
powershell -ExecutionPolicy Bypass -File build.ps1 -SkipZip
```

The binary is produced at `bin/artifacts/Release/spacecal_replay.exe`.

## Invocation

```
spacecal_replay <log.csv> [--mode oneshot|continuous]
                          [--threshold 1.5]
                          [--max-rel-error 0.005]
                          [--ignore-outliers]
```

| Flag | Default | Meaning |
| --- | --- | --- |
| `--mode oneshot` | | Run a single `ComputeOneshot` after pushing every sample. Mirrors the one-time calibration the overlay performs in non-continuous mode. |
| `--mode continuous` | (default) | Call `ComputeIncremental` after every pushed sample. Mirrors the live continuous-calibration loop. |
| `--threshold N` | `1.5` | Maps to `CalCtx.continuousCalibrationThreshold`. The factor that a new candidate calibration must beat the prior estimate by, to be accepted. |
| `--max-rel-error N` | `0.005` | Maps to `CalCtx.maxRelativeErrorThreshold`. |
| `--ignore-outliers` | off | Tells the calc to drop outliers before solving. |

The tool prints a summary with the number of ticks replayed, accept/reject
counts, watchdog trigger count, and the final calibration if one was produced.
Exit code 0 indicates a valid final calibration; non-zero indicates a parse
error or no calibration was reached.

## CSV format (v2)

A v2 log begins with the format-version banner before any data:

```
# spacecal_log_v2
Timestamp,posOffset_rawComputed.x,...,calibrationApplied,ref_tx,ref_ty,ref_tz,ref_qw,ref_qx,ref_qy,ref_qz,tgt_tx,tgt_ty,tgt_tz,tgt_qw,tgt_qx,tgt_qy,tgt_qz,tick_phase
0.123,...,FULL,0.500000,1.700000,-0.300000,1.0,0.0,0.0,0.0,...
```

Comment lines beginning with `#` are skipped (the overlay writes free-form
annotations like `# [123.45] StartContinuousCalibration`). The replay tool
refuses any input that doesn't begin with a `# spacecal_log_v2` banner — older
v1 captures (which lacked raw poses) cannot be replayed and the strict check
prevents silently misinterpreting them.

### Columns added in v2

All translations are in meters; rotations are unit quaternions in `(w, x, y, z)`
order. They are emitted with full double precision so the tool can reconstruct
the exact `Sample` values that fed `CalibrationCalc::PushSample`.

| Column | Meaning |
| --- | --- |
| `ref_tx`, `ref_ty`, `ref_tz` | Reference device translation in world space |
| `ref_qw`, `ref_qx`, `ref_qy`, `ref_qz` | Reference device rotation quaternion (world space) |
| `tgt_tx`, `tgt_ty`, `tgt_tz` | Target device translation in world space |
| `tgt_qw`, `tgt_qx`, `tgt_qy`, `tgt_qz` | Target device rotation quaternion (world space) |
| `tick_phase` | One of `None`, `Begin`, `Rotation`, `Translation`, `Editing`, `Continuous`, `ContinuousStandby` — the value of `CalibrationContext::state` at the moment the row was written |

### Existing columns (unchanged from v1)

`Timestamp`, the `posOffset_*.{x,y,z}` triplets, `error_*`, `axisIndependence`,
`rotationConditionRatio`, `consecutiveRejections`, `computationTime`,
`jitterRef`, `jitterTarget`, `calibrationApplied`. These are derived metrics
computed by the live overlay and are not consumed by the replay tool — they're
kept so that other analysis scripts (e.g. plotting) still see the same data they
did before.

## Compatibility note

v2 is a wire-incompatible change vs v1: the column count and order changed.
External CSV parsers that hard-coded v1's layout will need updating. The
overlay's own metrics writer was the only known consumer of v1 at the time of
the change.
