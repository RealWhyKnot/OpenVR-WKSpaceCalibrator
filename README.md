# WKOpenVR-SpaceCalibrator

Source has moved to [WKOpenVR](https://github.com/RealWhyKnot/WKOpenVR), the umbrella repository for the WKOpenVR feature modules.

This repo is kept as a release mirror. Each release here ships both a `WKOpenVR-Calibration-<version>.zip` (the WKOpenVR umbrella build with `enable_calibration.flag` pre-dropped so calibration activates immediately on install) and a `WKOpenVR-Calibration-v<version>-Setup.exe` (NSIS installer that does the same plus a Start Menu shortcut and an uninstaller).

## Upstream

This module is descended from [pushrax/OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) by Justin Li (MIT). The fork chain that reached this repository was:

- [pushrax/OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) -- original (Justin Li)
- [bdunderscore/OpenVR-SpaceCalibrator](https://github.com/bdunderscore/OpenVR-SpaceCalibrator)
- [ArcticFox8515/OpenVR-SpaceCalibrator](https://github.com/ArcticFox8515/OpenVR-SpaceCalibrator)
- [hyblocker/OpenVR-SpaceCalibrator](https://github.com/hyblocker/OpenVR-SpaceCalibrator) by Hyblocker -- the fork that was directly used to seed this work

The [`archive-source`](https://github.com/RealWhyKnot/WKOpenVR-SpaceCalibrator/tree/archive-source) branch on this repo preserves the original upstream `pushrax/OpenVR-SpaceCalibrator` source so the attribution chain stays intact.

Full credits and a side-by-side of which pieces are derived vs new in WKOpenVR live in the umbrella repo's [README](https://github.com/RealWhyKnot/WKOpenVR#upstreams-and-credits).

## Issues and contributions

Open them at [WKOpenVR/issues](https://github.com/RealWhyKnot/WKOpenVR/issues).

## License

GPL-3.0 (see LICENSE) for the combined work in this repository. The upstream and fork-chain contributions remain available under MIT from their origin repos.
