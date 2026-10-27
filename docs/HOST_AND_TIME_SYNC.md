# Host FSYNC and PTP configuration (snapshot, not installation instructions)

Collected from Jetson AGX Orin Host on 2026-10-26.

## FSYNC tool

- Source: `host_setup/fsync_tools/tsc_trigger.c`.
- The tool sends `CDI_TSC_FSYNC` ioctl to `/dev/cdi_tsc` to enable/disable an **already configured** generator.
- Source code does **not itself** set 30 Hz, 30% duty cycle, pinmux, or generator selection; those depend on board-level configuration and driver.
- Example build in an appropriate Jetson environment: `gcc -O2 -Wall -Wextra -o tsc_trigger host_setup/fsync_tools/tsc_trigger.c`.
- Do not run the tool before confirming that the correct `/dev/cdi_tsc`, DT `generator@480`, TSC_EDGE_OUT2, 1.8V pinmux and MAX9296 MFP1 route are configured; wrong board settings may affect connected cameras.

## PTP units

The files under `host_setup/systemd/` correspond to:

- `lidar-time-sync.target` requiring the two services.
- `mid360-ptp4l.service` invoking `ptp4l -f /etc/linuxptp/ptp4l-mid360-master.conf -m`.
- `mid360-phc2sys.service` invoking `phc2sys -s CLOCK_REALTIME -c eno1 -O 0 -m`.

`host_setup/linuxptp/ptp4l-mid360-master.conf` specifies hardware timestamping, UDPv4, E2E, domain 0, and `masterOnly 1` for **eno1**. Host system clock is the reference for PHC synchronization in these services. `/dev/ptp0` is assumed by the service precheck.

These are captured **live-system settings**. They are not generic settings for every Jetson/network interface and are not automatically enabled by this repository. Do not install or enable services until the original board configuration and network interface are verified.

## Still missing

- The actual Orin pinmux + pad-voltage files and applied DT overlay/patch.
- The MAX9296/Tier IV driver modification and the exact camera SDK driver version/install procedure.
- Reproducible Docker image definition, device access requirements and ROS 2 dependencies.
- Hardware timing test logs supporting 30 Hz/phase verification (available in reports; not included in this source snapshot).
