# SteamVR Overlay (this repo)
add_subdirectory ("overlay")

# SteamVR Driver (built out-of-tree by the OpenVR-PairDriver submodule).
# The driver source moved to RealWhyKnot/OpenVR-PairDriver and is built by
# its own CMake project under lib/OpenVR-PairDriver/. quick.ps1 -DeployDriver
# invokes the submodule's build.ps1 directly; this CMake tree no longer
# compiles a local driver target.
