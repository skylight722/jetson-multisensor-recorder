# Docker environment — not yet reproduced

The original recording ran inside `elated_satoshi` using image
`lidar-camera-projection:integrated-jpeg-final-20260725` on a Jetson AGX Orin.
The container had ROS 2 Humble and a workspace at
`/home/misys/forza_ws/race_stack` (with a `state_estimation` overlay).

An **older base-image** Dockerfile found at
`/home/misys/misys-f1fifth/build_context/Dockerfile` identifies itself as a
rebuild of the older `misys:3D_xvn` container. It starts from
`nvcr.io/nvidia/l4t-jetpack:r36.3.0`, while the documented target recording
environment uses JetPack 6.2.1 / L4T R36.4.7. Layer comparison on the original Orin confirmed that the final image
reuses all 67 root filesystem layers of `misys:3D_xvn` as its prefix
and adds one new layer (68 total). This **establishes image lineage**,
not a reproducible final build recipe.

We deliberately do not publish that historical Dockerfile as a usable
`Dockerfile`. The reviewed copy has a redacted user/password command and
also requires `packages.apt`, `requirements.pip`, and other build-context
files that have not yet been examined. It attempts to install dependencies
in a best-effort fashion (ignoring errors) and includes old project source.

To make this repository reproducible, capture the **actual** image's
build provenance (or construct a minimal ROS 2 Humble + NVIDIA GStreamer
image from tested dependencies), verify versions, install our 4 ROS 2
packages and the externally maintained Livox package, then test build and
a full recording on a staging system. Do not replace the working container.

See [deployment requirements](../docs/REPRODUCIBILITY_GAPS.md).
