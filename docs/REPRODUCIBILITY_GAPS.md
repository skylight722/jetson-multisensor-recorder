# Remaining gaps before a public reproducible release

The archived V6 source is an **unverified staging copy**, not a tested
standalone deployment. Its baseline hardware implementation remains intact.

1. **Docker image**: The candidate Dockerfile belongs to older
   `misys:3D_xvn`, which is confirmed as the first 67 of 68 layers of
   `lidar-camera-projection:integrated-jpeg-final-20260725`. Its dependency
   manifests are missing and the extra layer is not reconstructed; do not
   advertise this historical Dockerfile as a working final-image build.
2. **Livox driver source**: Installed package `livox_ros_driver2` v1.0.0,
   launch parameters and sanitized configuration are known. Source commit,
   actual network configuration and modifications are not.
3. **Hardware boot configuration**: NVIDIA Device Tree/pinmux,
   `generator@480`, MAX9296 driver patch, Tier IV SDK/install instructions
   must be documented and checked for redistribution rights.
4. **Calibration**: Real four-camera intrinsics, timing, extrinsics and
   correct mapping must be deployed privately or supplied as explicit
   templates. Present file names are baseline-specific.
5. **Environment configuration**: user-home-dependent paths have some
   overrides, but container identity, `eno1`, `/dev/nvpps0`, I2C device
   addresses, camera ordering and mounts are deployment-specific.
6. **Licenses**: Tier IV `ros2_v4l2_camera` retains Apache-2.0 obligations;
   NVIDIA `nvpps_ioctl.h` has GPL-2.0-only notice; other authorship and
   any required approvals must be resolved before blanket licensing.
7. **Verification**: no isolated ROS 2 Humble build or source-checkout
   camera/LiDAR/PTP/FSYNC recording run has been completed with this staging
   tree. The original tested recording metrics must not be attributed to it.

Further Docker limitations are documented in `docker/README.md`.
