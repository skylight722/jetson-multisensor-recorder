# Jetson Multi-Sensor Recorder

A ROS 2 recording stack for four Tier IV C2 cameras and a Livox MID-360 on the Jetson AGX Orin. It was developed to collect camera, LiDAR and IMU data with consistent timestamps for offline analysis.

The repository contains the recording scripts, a modified V4L2 camera driver, ROS 2 support packages and host-side time synchronization tools used in the V6 recording system.

## Overview

The recorder brings several sensor streams into ROS 2 and stores them in a single MCAP dataset:

- **Cameras:** Four Tier IV C2 (IMX490) cameras, triggered at 30 Hz. The modified camera node separates frame capture from image processing, resizes the images and publishes JPEG-compressed frames.
- **LiDAR and IMU:** Livox MID-360 data recorded at nominal rates of 10 Hz and 200 Hz, respectively.
- **Time synchronization:** TSC-generated camera FSYNC, per-camera ReadOutDelay settings, PTP for the LiDAR, and NvPPS timestamp correlation.
- **Recording and validation:** `rosbag2` with MCAP storage, followed by checks of message counts, rates, timestamps, frame sequences and recording coverage.

```text
4 × Tier IV C2 cameras ── V4L2 → resize / JPEG ─┐
Livox MID-360 ─────────── LiDAR / IMU ───────────┼─→ ROS 2 → rosbag2 (MCAP)
TSC / PTP / NvPPS ─────── timing metadata ──────┘                 │
                                                             validation
```

## Platform

| Component | Configuration |
| --- | --- |
| Computer | Jetson AGX Orin Devkit |
| OS / SDK | Ubuntu 22.04, JetPack 6.2.1 |
| ROS | ROS 2 Humble (Docker) |
| Cameras | 4 × Tier IV C2 (IMX490), 2880 × 1860 UYVY at 30 fps |
| Recorded images | 1920 × 1240 JPEG, quality 95 |
| Camera pipeline | V4L2 capture thread + processing worker, queue capacity 8 |
| LiDAR / IMU | Livox MID-360, 10 Hz / 200 Hz (nominal) |
| Output | 15 ROS 2 topics recorded in MCAP |

The camera conversion pipeline uses `nvvidconv` and `nvjpegenc` on the Jetson platform. The recorder also saves configuration snapshots, logs and validation results alongside each dataset.

## Repository structure

```text
recording_stack/   Recording control, camera launch and dataset validation
ros2_ws/src/       Camera driver and supporting ROS 2 packages
host_setup/        FSYNC source and PTP service configuration
config/            Example deployment and Livox settings
scripts/           Source checks and environment inspection
docs/              Hardware, dependencies and deployment notes
```

The main recording entry point is `recording_stack/time_sync_stack_v6_fsync.sh`. It handles FSYNC checks and delegates the recording sequence to `time_sync_stack_v6_core.sh`. Dataset validation is implemented in `finalize_offline_dataset_v6_fsync.py`.

## Usage

On the **original, configured Orin system**, recording is controlled from the host:

```bash
cd /path/to/lidar_camera_recording_stack

./time_sync_stack_v6_fsync.sh precheck
./time_sync_stack_v6_fsync.sh record 60 test_run
./time_sync_stack_v6_fsync.sh status
```

The recording command above requests a 60-second run named `test_run`. After the recording finishes, the finalizer writes its detailed report to `validation/result.txt` and marks the dataset `READY` or `NOT_READY` according to the recording checks.

These commands describe the deployed V6 workflow. **This reorganized repository has not yet been tested as a fresh installation.** Before using it on another Jetson, the Docker environment, camera mapping, calibration files and board-specific FSYNC settings must be configured and tested.

For a syntax and package-metadata check that does not require a Jetson:

```bash
bash scripts/check_source_snapshot.sh
```

## Reference recording

In a 300-second recording test on 2026-08-24 using the original Orin deployment:

| Metric | Observed result |
| --- | --- |
| Camera rate | Approximately 29.999 Hz per camera |
| JPEG frames | 9,122–9,123 per camera in the measured capture window |
| Common usable sensor interval | 303.047 s |
| Missing camera frame sequences during bag recording | 0 |
| Increase in queue overflow / JPEG encoding failures during bag recording | 0 |
| Final dataset check | `READY` |

These measurements are from the original recording environment; the reorganized source release has not yet been independently tested on hardware.

## Deployment notes

The current source is provided for inspection and further development. A clean deployment still requires the Tier IV camera SDK and board-level pinmux/Device Tree/MAX9296 setup, a reproducible Docker environment, camera calibration files and a matching Livox driver installation. The original Livox installation reported `livox_ros_driver2` version 1.0.0.

Details are collected in:

- [Deployment environment](docs/DEPLOYMENT_ENVIRONMENT.md)
- [Host FSYNC and PTP configuration](docs/HOST_AND_TIME_SYNC.md)
- [Livox driver dependency](docs/LIVOX_DEPENDENCY.md)
- [Reproducibility notes](docs/REPRODUCIBILITY_GAPS.md)

## Third-party code

The camera package is derived from [Tier IV's `ros2_v4l2_camera`](https://github.com/tier4/ros2_v4l2_camera) (`galactic`, base commit `ed965b487b8d23ebaa477a9eb2c6ce4cf3c151a0`) and retains its Apache-2.0 license. The NvPPS package includes an NVIDIA header marked `GPL-2.0-only`. Existing copyright and license notices must be preserved; a repository-wide license has not been assigned while the remaining provenance is reviewed.

See [ROS 2 package provenance and licenses](docs/ROS2_PACKAGES_AND_LICENSING.md).
