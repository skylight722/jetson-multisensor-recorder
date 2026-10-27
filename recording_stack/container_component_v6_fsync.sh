#!/usr/bin/env bash
set -Eeuo pipefail

COMPONENT=${1:?component required}
RUN=${2:?run directory required}
WS=${3:?workspace required}
export WS  # Shared by the camera and exposure launch scripts.
CAMERA_SCRIPT=${4:?camera script required}

STACK_DIR=$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &&
  pwd
)

QOS_OVERRIDES="$STACK_DIR/rosbag_qos_overrides_v6.yaml"
EXPOSURE_SCRIPT="$STACK_DIR/start_4cam_exposure_samplers_v6.sh"

shift 4

mkdir -p "$RUN/runtime"
printf '%s\n' "$$" > "$RUN/runtime/${COMPONENT}.pgid"

finish()
{
  local rc=$?
  trap - EXIT
  printf '%s\n' "$rc" > "$RUN/runtime/${COMPONENT}.exit_code"
  exit "$rc"
}
trap finish EXIT

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$WS/state_estimation/install/setup.bash" ]]; then
  source "$WS/state_estimation/install/setup.bash"
fi
source "$WS/install/setup.bash"
set -u

case "$COMPONENT" in
  nvpps)
    stdbuf -oL -eL \
      ros2 run nvpps_ros_publisher nvpps_ros_publisher \
        --ros-args \
        -r __ns:=/time_sync \
        -p device_path:=/dev/nvpps0 \
        -p poll_period_ms:=20 \
      >"$RUN/logs/nvpps.log" 2>&1
    ;;

  livox)
    stdbuf -oL -eL \
      ros2 launch livox_ros_driver2 msg_MID360_launch.py \
      >"$RUN/logs/livox.log" 2>&1
    ;;

  cameras)
    stdbuf -oL -eL \
      bash "$CAMERA_SCRIPT" "$RUN/logs/cameras" \
      >"$RUN/logs/camera_stack.log" 2>&1
    ;;

  exposure)
    mkdir -p "$RUN/logs/exposure"

    stdbuf -oL -eL \
      bash "$EXPOSURE_SCRIPT" "$RUN/logs/exposure" \
      >"$RUN/logs/exposure_stack.log" 2>&1
    ;;

  recorder)
    START_PAUSED=${1:-false}
    BAG_PATH=${2:?bag path required}
    shift 2

    if [[ ! -f "$QOS_OVERRIDES" ]]; then
      echo "Missing QoS override file: $QOS_OVERRIDES" >&2
      exit 2
    fi

    mkdir -p "$RUN/config"

    cp -a "$QOS_OVERRIDES" "$RUN/config/rosbag_qos_overrides.yaml"

    cat > "$RUN/config/recorder_settings.yaml" <<EOF
storage: mcap
max_cache_size_bytes: 2147483648
qos_profile_overrides:
  source: "$QOS_OVERRIDES"
  snapshot: config/rosbag_qos_overrides.yaml
EOF

    args=(
      --storage mcap
      --max-cache-size 2147483648
      --qos-profile-overrides-path "$QOS_OVERRIDES"
      -o "$BAG_PATH"
    )

    if [[ "$START_PAUSED" == "true" ]]; then
      args+=(--start-paused)
    fi

    RECORDER_STDIN="$RUN/runtime/recorder.stdin"

    rm -f "$RECORDER_STDIN"
    mkfifo "$RECORDER_STDIN"

    # Keep both FIFO ends open to avoid blocking and premature EOF.
    exec 9<>"$RECORDER_STDIN"

    recorder_cmd=(ros2 bag record "${args[@]}" "$@")
    printf -v recorder_cmd_q '%q ' "${recorder_cmd[@]}"

    # rosbag2 uses terminal input for pause/resume. Run it in a PTY
    # and send the resume key through the FIFO.
    set +e
    script -qefc "$recorder_cmd_q" /dev/null \
      <"$RECORDER_STDIN" \
      >"$RUN/logs/recorder.log" 2>&1
    recorder_rc=$?
    set -e

    exec 9>&-
    rm -f "$RECORDER_STDIN"

    exit "$recorder_rc"
    ;;

  *)
    echo "Unknown component: $COMPONENT" >&2
    exit 2
    ;;
esac
