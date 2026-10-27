# Livox driver: installed-runtime evidence

In the verified Orin recording container `elated_satoshi`, the package
`livox_ros_driver2` reports version `1.0.0` and is installed in an
**isolated** ROS 2 overlay at:

`/home/misys/forza_ws/race_stack/state_estimation/install/livox_ros_driver2`

The V6 component sources `$WS/state_estimation/install/setup.bash` before
`$WS/install/setup.bash`, then executes:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

The copied *installed* launch has: `xfer_format = 1` (Livox custom point
messages), `multi_topic = 0`, `data_src = 0`, `publish_freq = 10.0`,
`frame_id = livox_frame`; it uses the adjacent installed
`config/MID360_config.json`.

The received JSON was sanitized by replacing every IPv4 address with
`0.0.0.0`. It must **not** be used for hardware operation. For public
documentation, see `config/livox/MID360_config.example.json`, which has
explicit address placeholders and copies non-secret port/settings fields.
Confirm sensor addresses, source version and installation separately.

`package.xml` version 1.0.0 is not a precise Git commit. We have **not**
confirmed if the ROS 2 Livox driver was changed locally. Do not blindly
vendor installed generated CMake files, message code or ROS binaries.
