#!/usr/bin/env bash
set -Eeuo pipefail

LOG_DIR=${1:?log directory required}

mkdir -p "$LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
WS=${WS:-$HOME/forza_ws/race_stack}
source "$WS/install/setup.bash"
set -u

PIDS=()

cleanup()
{
    trap - INT TERM EXIT

    for pid in "${PIDS[@]:-}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done

    for pid in "${PIDS[@]:-}"; do
        wait "$pid" 2>/dev/null || true
    done
}

trap cleanup INT TERM EXIT

start_sampler()
{
    local cam=$1
    local device=$2

    echo "Starting $cam exposure sampler on $device"

    ros2 run \
      c2_exposure_sampler \
      c2_exposure_sampler_node \
      --ros-args \
      -r __ns:=/camera/"$cam" \
      -r __node:=c2_exposure_"$cam" \
      -p camera_name:="$cam" \
      -p i2c_device:="$device" \
      -p i2c_address:=109 \
      -p max_attempts:=3 \
      -p retry_delay_us:=1000 \
      >"$LOG_DIR/${cam}.log" 2>&1 &

    PIDS+=("$!")
}

start_sampler cam0 /dev/i2c-9
start_sampler cam1 /dev/i2c-10
start_sampler cam2 /dev/i2c-11
start_sampler cam3 /dev/i2c-12

echo "All four C2 exposure samplers started"
printf 'PIDs: %s\n' "${PIDS[*]}"

wait
