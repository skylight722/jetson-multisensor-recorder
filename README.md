# Jetson multi-sensor recorder

ROS 2 recording stack for four Tier IV C2 cameras and a Livox MID-360 on a Jetson AGX Orin. The system was developed to capture camera, LiDAR and IMU data together while retaining the timing information needed for offline sensor processing.

This repository contains the V6 recording scripts and modified camera driver. It is a **source release candidate**, not a one-command installer: board-level camera configuration, the original Docker environment and site-specific calibration files still need to be reconstructed and tested.

## System overview

```text
Tier IV C2 cameras (4 × 30 fps)
    └─ V4L2 capture → bounded queue → resize + JPEG → ROS 2
Livox MID-360 (LiDAR 10 Hz, IMU 200 Hz) ──────────────→ ROS 2
TSC / NvPPS / PTP timing information ─────────────────→ ROS 2
                                                       │
                                               rosbag2 / MCAP
                                                       │
                                              dataset validation
```

Each camera runs in a separate `v4l2_camera_node` process. The modified driver separates V4L2 acquisition from image processing and publishes `sensor_msgs/CompressedImage` directly. The recording process checks required topics before starting the recorder, then verifies the resulting dataset.

### Recording configuration

| Component | Configuration |
| --- | --- |
| Compute | Jetson AGX Orin Devkit, JetPack 6.2.1 / Ubuntu 22.04 |
| Middleware | ROS 2 Humble |
| Cameras | 4 × Tier IV C2 (IMX490), 2880 × 1860 UYVY, 30 fps |
| Recorded images | 1920 × 1240 JPEG (quality 95) |
| Camera processing | One capture thread and one processing worker per camera; queue capacity 8 |
| LiDAR / IMU | Livox MID-360, nominally 10 Hz / 200 Hz |
| Timing | TSC-generated 30 Hz FSYNC, camera ReadOutDelay, PTP and NvPPS |
| Storage | rosbag2 with MCAP, 2 GiB recorder cache |
| Recorded topics | 15 required topics: images, timestamp/exposure debug, LiDAR, IMU and NvPPS |

## Source layout

```text
recording_stack/   Recording commands, camera launch, QoS settings and validation
ros2_ws/src/       Modified camera driver and three supporting ROS 2 packages
host_setup/        TSC trigger source and captured linuxptp/systemd configuration
config/            Example deployment settings and Livox configuration template
scripts/           Source checks (no hardware access)
docs/              Hardware requirements, dependencies and outstanding setup work
```

The principal entry point is `recording_stack/time_sync_stack_v6_fsync.sh`. It manages FSYNC and delegates the recording lifecycle to `time_sync_stack_v6_core.sh`. After recording, `finalize_offline_dataset_v6_fsync.py` checks message rates, timestamps, camera frame sequences, JPEG data, timing coverage and other recording conditions.

## Commands

On the **configured original Orin environment**, the host-side commands are:

```bash
cd /path/to/lidar_camera_recording_stack
./time_sync_stack_v6_fsync.sh precheck
./time_sync_stack_v6_fsync.sh record 60 test_run
./time_sync_stack_v6_fsync.sh status
```

These are the V6 commands, not verified installation instructions for this reorganized checkout. Do not run the copied scripts against the active recording system without reviewing the deployment paths, container mounts, camera calibration and device configuration first.

The recorder writes a run directory containing the MCAP bag, calibration/configuration snapshots, timing metadata, log files, a validation report and a `READY` or `NOT_READY` marker. The full validation summary is saved at `validation/result.txt`; the terminal shows a shorter overview.

## Reference recording

A 300-second test on the **original deployed stack** (2026-08-24) reported approximately 29.999 Hz per camera, 9,122–9,123 JPEG frames per camera and a 303.047-second common usable interval. During the bag recording interval, the measured camera sequence-missing count and increases in queue overflow and encode failures were zero. The finalizer reported `READY`.

These results describe that tested deployment. They have not yet been reproduced with the files in this repository.

## Before a clean deployment

The current snapshot depends on the following items, which are **not fully provided** here:

- Tier IV camera SDK and the board's pinmux / Device Tree / MAX9296 modifications for FSYNC.
- A reproducible Docker image. The original recording image adds a layer to an older development image; its complete build recipe is still being recovered.
- A pinned Livox driver source revision and device-specific network settings. The installed package was `livox_ros_driver2` 1.0.0.
- The four cameras' calibration YAML files and matching `/dev/video*` device mapping.

The files under `host_setup/` capture one board's settings; they are not portable installation scripts. See [deployment notes](docs/DEPLOYMENT_ENVIRONMENT.md), [hardware and time synchronization](docs/HOST_AND_TIME_SYNC.md), and [remaining requirements](docs/REPRODUCIBILITY_GAPS.md).

For source-only checks, run:

```bash
bash scripts/check_source_snapshot.sh
```

This checks syntax and package metadata. It does not build ROS 2 packages or test recording hardware.

## Upstream code and licenses

The camera package is based on [Tier IV's `ros2_v4l2_camera`](https://github.com/tier4/ros2_v4l2_camera) (`galactic`, base commit `ed965b487b8d23ebaa477a9eb2c6ce4cf3c151a0`) and retains its upstream Apache-2.0 license. The NvPPS package contains an NVIDIA header marked `GPL-2.0-only`. These notices must be preserved. Ownership and redistribution permissions for the rest of the project still need review before applying a repository-wide license.

See [package provenance and licensing](docs/ROS2_PACKAGES_AND_LICENSING.md).
