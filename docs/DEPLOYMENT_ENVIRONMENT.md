# Environment parameters and remaining platform-specific settings

This is a source-only staging copy; **do not deploy or run it on a production Orin** before a separate ROS 2 build and controlled test.

## Host environment (shell)

`config/deployment.env.example` is an illustrative environment file and is not sourced automatically. `STACK` defaults to the V6 wrapper script directory when unset. `TSC_TRIGGER` defaults to `$HOME/fsync_tools/tsc_trigger`; `WS`, `DATA_ROOT` default to `$HOME/forza_ws/race_stack`, `$HOME/shared_dir/offline_projection_runs`. `CONTAINER` still defaults to `elated_satoshi`. Set these in the invoking Host process before calling the wrapper.

## Docker environment

`container_component_v6_fsync.sh` now exports the `WS` argument to its camera/exposure child scripts. Camera intrinsics are still the original four *specific YAML filenames*, under `CAMERA_INTRINSICS_ROOT` (default `$HOME/shared_dir/camera_calibration/intrinsics` **inside the container**). The finalizer uses `CAMERA_CALIBRATION_BUNDLE_ROOT` (default `$HOME/shared_dir/camera_calibration/final_4cam` **inside the container**). Simply exporting these on the Host will **not** automatically pass them through `docker exec`. For a new deployment, adapt Docker environment and mounts deliberately.

## Hardware-specific dependencies not generalized

Device names (`/dev/nvpps0`, `eno1`, `/dev/i2c-9..12`), camera port mapping, ReadOutDelay values, camera sensor names, `tier4_imx490` modules and MAX9296 patch are deliberately unchanged. Their correctness is board- and driver-specific and must not be guessed. `tsc_trigger.c` requires the previously configured TSC device-tree, pinmux and installed driver; it is not a standalone FSYNC solution.

## Changes compared with the captured baseline

- Script base directory and home-relative defaults instead of literal `/home/misys` path defaults.
- Container component propagates `WS` to its child scripts.
- Host `df` commands use configured `DATA_ROOT` rather than an unrelated fixed shared directory.
- Camera intrinsics *root* and finalizer calibration *bundle root* can be set via container-side environment variables; filenames remain pinned to the validated setup.

This is a **refactor not a proven replacement**: static syntax only, no ROS build, FSYNC/PTP operation, LiDAR capture or 300-second test performed on this reorganized project.
