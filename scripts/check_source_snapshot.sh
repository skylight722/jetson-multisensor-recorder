#!/usr/bin/env bash
# Static-only checks. Does NOT access sensors, start Docker, change clocks, or record data.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

req_scripts=(
  time_sync_stack_v6_fsync.sh
  time_sync_stack_v6_core.sh
  container_component_v6_fsync.sh
  start_4cams_integrated_jpeg_q95_offline.sh
  start_4cam_exposure_samplers_v6.sh
  apply_c2_readout_delay_runtime.sh
)
for script in "${req_scripts[@]}"; do
  bash -n "$ROOT/recording_stack/$script"
  echo "PASS shell syntax: $script"
done

python3 - "$ROOT" <<'PY'
import ast
import pathlib
import sys
import xml.etree.ElementTree as ET
root=pathlib.Path(sys.argv[1])
f=root/'recording_stack/finalize_offline_dataset_v6_fsync.py'
ast.parse(f.read_text(encoding='utf-8'),filename=str(f))
print(f'PASS Python syntax: {f.name}')
import json
conf=json.loads((root/'config/livox/MID360_config.example.json').read_text())
assert conf['lidar_configs'][0]['ip']=='SET_MID360_IP'
assert conf['MID360']['host_net_info']['point_data_ip']=='SET_HOST_IP'
print('PASS Livox example: JSON schema parse and explicit IP placeholders')
for directory, expected in [('ros2_v4l2_camera','v4l2_camera'), ('time_sync_msgs','time_sync_msgs'), ('nvpps_ros_publisher','nvpps_ros_publisher'), ('c2_exposure_sampler','c2_exposure_sampler')]:
    p=root/'ros2_ws/src'/directory/'package.xml'
    e=ET.parse(p).getroot()
    actual=e.findtext('name')
    assert actual==expected,(p,actual,expected)
    print(f'PASS package.xml: {directory} (ROS package {actual})')
PY

if command -v gcc >/dev/null 2>&1; then
  gcc -std=c11 -Wall -Wextra -Wpedantic -fsyntax-only "$ROOT/host_setup/fsync_tools/tsc_trigger.c"
  echo 'PASS C syntax: tsc_trigger.c'
else
  echo 'SKIP C syntax: gcc not installed'
fi

echo 'Static source inspection completed. No ROS build or device test was run.'
