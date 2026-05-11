# SteamVR Overlay (this repo)
add_subdirectory ("overlay")

# SteamVR Driver (built out-of-tree by the OpenVR-WKPairDriver submodule).
# The driver source moved to RealWhyKnot/OpenVR-WKPairDriver and is built by
# its own CMake project under lib/OpenVR-WKPairDriver/. quick.ps1 -DeployDriver
# invokes the submodule's build.ps1 directly; this CMake tree no longer
# compiles a local driver target.
