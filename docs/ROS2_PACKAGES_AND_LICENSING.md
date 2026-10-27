# ROS 2 packages, dependencies and license review

## Files collected

1. `ros2_v4l2_camera`: modified Tier IV camera node. Original upstream Apache-2.0 LICENSE is retained. Tracked changes from upstream base and newly added JPEG source are present in the snapshot.
2. `time_sync_msgs`: `CameraTimestampDebug.msg`, `NvPpsCorrelation.msg`.
3. `nvpps_ros_publisher`: publisher using `/dev/nvpps0` and `NVPPS_GETEVENT`.
4. `c2_exposure_sampler`: message and node using `/dev/i2c-9` through `/dev/i2c-12` in the current launch scripts.

All four packages have `package.xml`; this is **not** proof of a clean independent ROS 2 Humble build. `c2_exposure_sampler` generates `CameraExposureDebug.msg` in its own CMake build.

## Third-party sources and licensing checks before publication

- Camera package is derived from https://github.com/tier4/ros2_v4l2_camera, `galactic`, base `ed965b487b8d23ebaa477a9eb2c6ce4cf3c151a0`. Keep upstream LICENSE and original attribution.
- `nvpps_ros_publisher/include/nvpps_ros_publisher/nvpps_ioctl.h` explicitly declares **SPDX-License-Identifier: GPL-2.0-only**, copyright NVIDIA (2022). `nvpps_ros_publisher/package.xml` declares Apache-2.0 for the package. Do not assume the entire copied package is uniformly Apache-2.0. Review obligations, file provenance, and distribution authorization; preserve the upstream header notice.
- `time_sync_msgs` and `c2_exposure_sampler` package manifests declare Apache-2.0 but contain no standalone LICENSE copies in the submitted archive. Confirm ownership and licensing, including lab approval, before posting publicly.
- Tier IV C2 SDK and NVIDIA board-specific driver/Device Tree files are **not** included. Review the relevant license before adding any proprietary/vendor files.
- Livox ROS driver is not included: installed `package.xml` version 1.0.0 identified in the existing `state_estimation/install` overlay, but exact tag/commit, `msg_MID360_launch.py` file, connected sensor config (IP/interface), and any local modifications must be checked. Use an exact pinned external dependency if unchanged rather than arbitrarily bundling external code.

## Security

Audit all source and configuration before public push, including Git history. The submitted files were screened for common credential markers, but this is not a comprehensive secrets audit. No original `.git` history is included in this draft.
