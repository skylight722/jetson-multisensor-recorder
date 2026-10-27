#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/humble/setup.bash
WS=${WS:-$HOME/forza_ws/race_stack}
source "$WS/install/setup.bash"
set -u

RUN=${1:-${DATA_ROOT:-$HOME/shared_dir/offline_projection_runs}/camera_run_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$RUN"

# Camera order is fixed: left (9-002b), front (10-002b),
# right (11-002b), rear (12-002b).
SENSOR_NAMES=(
  "vi-output, tier4_imx490 9-002b"
  "vi-output, tier4_imx490 10-002b"
  "vi-output, tier4_imx490 11-002b"
  "vi-output, tier4_imx490 12-002b"
)

DIRECTIONS=(left front right rear)
FRAMES=(camera_left camera_front camera_right camera_rear)

CAMERA_INTRINSICS_ROOT=${CAMERA_INTRINSICS_ROOT:-$HOME/shared_dir/camera_calibration/intrinsics}
INTRINSICS=(
  "$CAMERA_INTRINSICS_ROOT"/left_1920x1240_20260726/cam0_info.yaml
  "$CAMERA_INTRINSICS_ROOT"/front_1920x1240_20260726/cam1_info.yaml
  "$CAMERA_INTRINSICS_ROOT"/right_1920x1240_20260811_redo/cam2_info.yaml
  "$CAMERA_INTRINSICS_ROOT"/rear_1920x1240_20260726/cam3_info.yaml
)

find_video_device()
{
  local expected_name="$1"
  local sysfs_node actual_name video_name

  for sysfs_node in /sys/class/video4linux/video*; do
    [[ -r "${sysfs_node}/name" ]] || continue
    actual_name=$(<"${sysfs_node}/name")

    if [[ "$actual_name" == "$expected_name" ]]; then
      video_name=$(basename "$sysfs_node")
      printf '/dev/%s\n' "$video_name"
      return 0
    fi
  done

  echo "ERROR: Camera device not found: ${expected_name}" >&2
  return 1
}

DEVICES=()
PIDS=()

cleanup()
{
  trap - INT TERM EXIT
  echo
  echo "Stopping camera nodes"

  for pid in "${PIDS[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
  done

  for pid in "${PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}

trap cleanup INT TERM EXIT

echo "Camera devices"

for i in 0 1 2 3; do
  device=$(find_video_device "${SENSOR_NAMES[$i]}")
  DEVICES+=("$device")

  [[ -c "$device" ]] || {
    echo "ERROR: Detected path is not a video device: $device" >&2
    exit 1
  }

  [[ -f "${INTRINSICS[$i]}" ]] || {
    echo "ERROR: Intrinsic YAML not found: ${INTRINSICS[$i]}" >&2
    exit 1
  }

  echo "cam${i} (${DIRECTIONS[$i]}): ${SENSOR_NAMES[$i]} -> ${device}"
done

if [[ $(printf '%s\n' "${DEVICES[@]}" | sort -u | wc -l) -ne 4 ]]; then
  echo "ERROR: Duplicate camera device mapping detected." >&2
  exit 1
fi

cat > "$RUN/camera_mapping.yaml" <<EOF
cameras:
  cam0:
    direction: left
    frame_id: camera_left
    sensor_name: "${SENSOR_NAMES[0]}"
    device: "${DEVICES[0]}"
    intrinsics: "${INTRINSICS[0]}"
  cam1:
    direction: front
    frame_id: camera_front
    sensor_name: "${SENSOR_NAMES[1]}"
    device: "${DEVICES[1]}"
    intrinsics: "${INTRINSICS[1]}"
  cam2:
    direction: right
    frame_id: camera_right
    sensor_name: "${SENSOR_NAMES[2]}"
    device: "${DEVICES[2]}"
    intrinsics: "${INTRINSICS[2]}"
  cam3:
    direction: rear
    frame_id: camera_rear
    sensor_name: "${SENSOR_NAMES[3]}"
    device: "${DEVICES[3]}"
    intrinsics: "${INTRINSICS[3]}"
image:
  source_resolution: [2880, 1860]
  output_resolution: [1920, 1240]
  source_encoding: UYVY
  compressed_format: jpeg
  jpeg_quality: 95
timestamp_debug: true
EOF

echo "Log directory: $RUN"

for i in 0 1 2 3; do
  echo "Starting cam${i} (${DIRECTIONS[$i]}): ${DEVICES[$i]}"

  stdbuf -oL -eL \
    ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -r __node:=camera_node \
    -r __ns:=/camera/cam${i} \
    -p video_device:="${DEVICES[$i]}" \
    -p pixel_format:=UYVY \
    -p output_encoding:=yuv422 \
    -p image_size:="[2880,1860]" \
    -p enable_resize:=true \
    -p resize_size:="[1920,1240]" \
    -p resize_queue_capacity:=8 \
    -p publish_compressed_jpeg:=true \
    -p jpeg_quality:=95 \
    -p time_per_frame:="[1,30]" \
    -p use_v4l2_buffer_timestamps:=true \
    -p use_image_transport:=false \
    -p use_sensor_data_qos:=false \
    -p camera_frame_id:="${FRAMES[$i]}" \
    -p camera_info_url:="file://${INTRINSICS[$i]}" \
    -p enable_timestamp_debug:=true \
    >"$RUN/cam${i}.log" 2>&1 &

  PIDS+=("$!")

  if [[ "$i" -lt 3 ]]; then
    sleep 2
  fi
done

printf '%s\n' "${PIDS[@]}" > "$RUN/camera_node_pids.txt"

echo
echo "Camera nodes started: 4"
echo "Images: /camera/cam{0..3}/image_raw/compressed"
echo "Logs: $RUN"

wait
