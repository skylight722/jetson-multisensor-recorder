#!/usr/bin/env bash
set -Eeuo pipefail

STACK_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

CONTAINER=${CONTAINER:-elated_satoshi}
WS=${WS:-$HOME/forza_ws/race_stack}
DATA_ROOT=${DATA_ROOT:-$HOME/shared_dir/offline_projection_runs}
PTP_TARGET=${PTP_TARGET:-lidar-time-sync.target}
ACTIVE_FILE="$DATA_ROOT/.active_run"
LATEST_FILE="$DATA_ROOT/latest_run.txt"

CAMERA_SCRIPT="$STACK_DIR/start_4cams_integrated_jpeg_q95_offline.sh"
COMPONENT_SCRIPT="$STACK_DIR/container_component_v6_fsync.sh"
FINALIZER="$STACK_DIR/finalize_offline_dataset_v6_fsync.py"
READOUT_DELAY_SCRIPT="$STACK_DIR/apply_c2_readout_delay_runtime.sh"
EXPOSURE_SCRIPT="$STACK_DIR/start_4cam_exposure_samplers_v6.sh"

WARMUP_SECONDS=${WARMUP_SECONDS:-5}
BASE_TOPIC_COUNT=11
EXPOSURE_TOPIC_COUNT=4
READOUT_DELAY_ENABLED=${READOUT_DELAY_ENABLED:-true}
ANCHOR_GUARD_SECONDS=${ANCHOR_GUARD_SECONDS:-2}
RECORDER_SETTLE_SECONDS=${RECORDER_SETTLE_SECONDS:-2}
FSYNC_ENABLED=${FSYNC_ENABLED:-false}
FSYNC_SOURCE=${FSYNC_SOURCE:-none}
FSYNC_GENERATOR=${FSYNC_GENERATOR:-none}
FSYNC_FREQUENCY_HZ=${FSYNC_FREQUENCY_HZ:-0}
FSYNC_DUTY_CYCLE_PERCENT=${FSYNC_DUTY_CYCLE_PERCENT:-0}
FSYNC_TRIGGER_MODE=${FSYNC_TRIGGER_MODE:-0}
FSYNC_MFP=${FSYNC_MFP:-0}
FSYNC_RUNTIME_INFO=${FSYNC_RUNTIME_INFO:-}

TOPICS=(
  /camera/cam0/image_raw/compressed
  /camera/cam1/image_raw/compressed
  /camera/cam2/image_raw/compressed
  /camera/cam3/image_raw/compressed
  /camera/cam0/timestamp_debug
  /camera/cam1/timestamp_debug
  /camera/cam2/timestamp_debug
  /camera/cam3/timestamp_debug
  /livox/lidar
  /livox/imu
  /time_sync/nvpps
  /camera/cam0/exposure_debug
  /camera/cam1/exposure_debug
  /camera/cam2/exposure_debug
  /camera/cam3/exposure_debug
)

EXPECTED_TYPES=(
  sensor_msgs/msg/CompressedImage
  sensor_msgs/msg/CompressedImage
  sensor_msgs/msg/CompressedImage
  sensor_msgs/msg/CompressedImage
  time_sync_msgs/msg/CameraTimestampDebug
  time_sync_msgs/msg/CameraTimestampDebug
  time_sync_msgs/msg/CameraTimestampDebug
  time_sync_msgs/msg/CameraTimestampDebug
  livox_ros_driver2/msg/CustomMsg
  sensor_msgs/msg/Imu
  time_sync_msgs/msg/NvPpsCorrelation
  c2_exposure_sampler/msg/CameraExposureDebug
  c2_exposure_sampler/msg/CameraExposureDebug
  c2_exposure_sampler/msg/CameraExposureDebug
  c2_exposure_sampler/msg/CameraExposureDebug
)

log()
{
  printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

die()
{
  echo "ERROR: $*" >&2
  exit 1
}

sanitize_label()
{
  local value=${1:-run}
  value=${value// /_}
  value=$(printf '%s' "$value" | tr -cd '[:alnum:]_.-')
  [[ -n "$value" ]] || value=run
  printf '%s\n' "$value"
}

require_environment()
{
  command -v docker >/dev/null || die "docker command not found"
  command -v sudo >/dev/null || die "sudo command not found"
  [[ -f "$CAMERA_SCRIPT" ]] || die "Missing: $CAMERA_SCRIPT"
  [[ -f "$COMPONENT_SCRIPT" ]] || die "Missing: $COMPONENT_SCRIPT"
  [[ -f "$FINALIZER" ]] || die "Missing: $FINALIZER"
  [[ -x "$EXPOSURE_SCRIPT" ]] || die "Missing or not executable: $EXPOSURE_SCRIPT"

  (( ${#TOPICS[@]} == ${#EXPECTED_TYPES[@]} )) || die "TOPICS / EXPECTED_TYPES length mismatch"

  (( ${#TOPICS[@]} == BASE_TOPIC_COUNT + EXPOSURE_TOPIC_COUNT )) || die "Unexpected v6 required topic count"

  if [[ "$READOUT_DELAY_ENABLED" == "true" ]]; then
    [[ -x "$READOUT_DELAY_SCRIPT" ]] || die "Missing or not executable: $READOUT_DELAY_SCRIPT"
  fi

  mkdir -p "$DATA_ROOT"
}

docker_ros()
{
  local command=$1
  docker exec "$CONTAINER" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f '$WS/state_estimation/install/setup.bash' ]]; then
      source '$WS/state_estimation/install/setup.bash'
    fi
    source '$WS/install/setup.bash'
    set -u
    $command
  "
}

active_run()
{
  [[ -f "$ACTIVE_FILE" ]] || return 1
  local run
  run=$(<"$ACTIVE_FILE")
  [[ -n "$run" && -d "$run" ]] || return 1
  printf '%s\n' "$run"
}

create_run()
{
  local label
  label=$(sanitize_label "${1:-run}")
  local run="$DATA_ROOT/run_$(date +%Y%m%d_%H%M%S)_${label}"

  mkdir -p \
    "$run/logs/cameras" \
    "$run/validation" \
    "$run/config" \
    "$run/timing" \
    "$run/calibration" \
    "$run/runtime"

  printf '%s\n' "$run" > "$ACTIVE_FILE"
  printf '%s\n' "$run" > "$LATEST_FILE"

  {
    echo "created_at=$(date --iso-8601=seconds)"
    echo "hostname=$(hostname)"
    echo "container=$CONTAINER"
    echo "workspace=$WS"
    echo "fsync_enabled=$FSYNC_ENABLED"
    echo "fsync_source=$FSYNC_SOURCE"
    echo "fsync_generator=$FSYNC_GENERATOR"
    echo "fsync_frequency_hz=$FSYNC_FREQUENCY_HZ"
    echo "fsync_duty_cycle_percent=$FSYNC_DUTY_CYCLE_PERCENT"
    echo "fsync_trigger_mode=$FSYNC_TRIGGER_MODE"
    echo "fsync_mfp=$FSYNC_MFP"
    echo "readout_delay_enabled=$READOUT_DELAY_ENABLED"
    echo "readout_delay_apply_stage=after_camera_topic_gate_and_warmup"
    echo "camera_format=JPEG"
    echo "jpeg_quality=95"
    echo "resolution=1920x1240"
    echo "stack_version=6"
    echo "required_topic_count=15"
    echo "exposure_enabled=true"
    echo "exposure_source=gw5300_current_shutter"
    echo "exposure_trigger=timestamp_debug"
    echo "exposure_status_read_bytes=101"
    echo "exposure_raw_count_unit_ns=12500"
    echo "exposure_frame_association=provisional"
  } > "$run/config/runtime_environment.txt"

  if [[ -n "$FSYNC_RUNTIME_INFO" &&
        -f "$FSYNC_RUNTIME_INFO" ]]; then
    cp -a "$FSYNC_RUNTIME_INFO" "$run/config/fsync_runtime.env"
  fi

  df -h "$DATA_ROOT" > "$run/config/storage_before.txt" 2>&1 || true
  uname -a > "$run/config/uname.txt" 2>&1 || true
  docker inspect "$CONTAINER" \
    --format 'image={{.Config.Image}} status={{.State.Status}}' \
    > "$run/config/container.txt" 2>&1 || true

  docker exec "$CONTAINER" bash -lc "
    cd '$WS'
    git rev-parse HEAD 2>/dev/null || true
    git status --short 2>/dev/null || true
  " > "$run/config/git_state.txt" 2>&1 || true

  printf '%s\n' "$run"
}

known_processes()
{
  docker exec "$CONTAINER" bash -lc "
    pgrep -af \
      '[v]4l2_camera_node|[n]vpps_ros_publisher|[l]ivox_ros_driver2_node|[m]sg_MID360_launch.py|[s]tart_4cams_integrated_jpeg_q95_offline.sh|[c]2_exposure_sampler_node|[s]tart_4cam_exposure_samplers_v6.sh|[r]os2 bag record' \
      || true
  " 2>/dev/null
}

launch_component()
{
  local run=$1
  local name=$2
  shift 2

  rm -f \
    "$run/runtime/${name}.pgid" \
    "$run/runtime/${name}.exit_code"

  docker exec -d "$CONTAINER" \
    setsid bash "$COMPONENT_SCRIPT" \
    "$name" "$run" "$WS" "$CAMERA_SCRIPT" "$@"

  local deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    if [[ -s "$run/runtime/${name}.pgid" ]]; then
      local pgid
      pgid=$(<"$run/runtime/${name}.pgid")
      if docker exec "$CONTAINER" bash -lc \
        "kill -0 -- -'$pgid' 2>/dev/null"; then
        return 0
      fi
    fi
    sleep 0.2
  done

  die "$name did not create a live process group"
}

group_alive()
{
  local run=$1
  local name=$2
  local file="$run/runtime/${name}.pgid"
  [[ -s "$file" ]] || return 1
  local pgid
  pgid=$(<"$file")
  docker exec "$CONTAINER" bash -lc \
    "kill -0 -- -'$pgid' 2>/dev/null"
}

stop_component()
{
  local run=$1
  local name=$2
  local grace=${3:-10}
  local file="$run/runtime/${name}.pgid"

  [[ -s "$file" ]] || return 0

  local pgid
  pgid=$(<"$file")

  if ! docker exec "$CONTAINER" bash -lc \
    "kill -0 -- -'$pgid' 2>/dev/null"; then
    return 0
  fi

  docker exec "$CONTAINER" bash -lc \
    "kill -INT -- -'$pgid' 2>/dev/null || true" >/dev/null 2>&1 || true

  local deadline=$((SECONDS + grace))
  while (( SECONDS < deadline )); do
    if ! docker exec "$CONTAINER" bash -lc \
      "kill -0 -- -'$pgid' 2>/dev/null"; then
      return 0
    fi
    sleep 0.25
  done

  docker exec "$CONTAINER" bash -lc \
    "kill -TERM -- -'$pgid' 2>/dev/null || true" >/dev/null 2>&1 || true
  sleep 2

  if docker exec "$CONTAINER" bash -lc \
    "kill -0 -- -'$pgid' 2>/dev/null"; then
    docker exec "$CONTAINER" bash -lc \
      "kill -KILL -- -'$pgid' 2>/dev/null || true" >/dev/null 2>&1 || true
    sleep 1
  fi
}

cleanup_known_processes()
{
  if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
    return 0
  fi

  for signal in INT TERM KILL; do
    docker exec "$CONTAINER" bash -lc "
      pkill -'$signal' -f '[v]4l2_camera_node' 2>/dev/null || true
      pkill -'$signal' -f '[n]vpps_ros_publisher' 2>/dev/null || true
      pkill -'$signal' -f '[l]ivox_ros_driver2_node' 2>/dev/null || true
      pkill -'$signal' -f '[m]sg_MID360_launch.py' 2>/dev/null || true
      pkill -'$signal' -f '[s]tart_4cams_integrated_jpeg_q95_offline.sh' 2>/dev/null || true
      pkill -'$signal' -f '[c]2_exposure_sampler_node' 2>/dev/null || true
      pkill -'$signal' -f '[s]tart_4cam_exposure_samplers_v6.sh' 2>/dev/null || true
      pkill -'$signal' -f '[r]os2 bag record' 2>/dev/null || true
    " >/dev/null 2>&1 || true

    [[ "$signal" == "INT" ]] && sleep 3
    [[ "$signal" == "TERM" ]] && sleep 2
  done
}

wait_for_topics()
{
  local run=$1
  local required_count=${2:-${#TOPICS[@]}}
  local deadline=$((SECONDS + 60))
  local listing=""
  local missing

  while (( SECONDS < deadline )); do
    listing=$(docker_ros "ros2 topic list -t" 2>/dev/null || true)
    missing=0

    local i
    for ((i = 0; i < required_count; ++i)); do
      if ! grep -Fq \
        "${TOPICS[$i]} [${EXPECTED_TYPES[$i]}]" \
        <<<"$listing"; then
        missing=$((missing + 1))
      fi
    done

    if (( missing == 0 )); then
      if (( required_count == BASE_TOPIC_COUNT )); then
        printf '%s
' "$listing" \
          > "$run/validation/topic_list_base_after_start.txt"
      else
        printf '%s
' "$listing" \
          > "$run/validation/topic_list_after_start.txt"
      fi
      return 0
    fi

    log \
      "Waiting for required topics... " \
      "required=$required_count missing=$missing"

    sleep 2
  done

  printf '%s
' "$listing" \
    > "$run/validation/topic_list_start_failure.txt"

  return 1
}


parallel_message_preflight()
{
  local run=$1
  local topic_args=""
  for topic in "${TOPICS[@]}"; do
    printf -v topic_args '%s %q' "$topic_args" "$topic"
  done

  set +e
  docker_ros "
    set +e
    topics=($topic_args)
    pids=()
    for topic in \"\${topics[@]}\"; do
      timeout 12s ros2 topic echo \"\$topic\" --once >/dev/null 2>&1 &
      pids+=(\"\$!\")
    done

    rc=0
    for i in \"\${!pids[@]}\"; do
      if ! wait \"\${pids[\$i]}\"; then
        echo \"NO_MESSAGE \${topics[\$i]}\"
        rc=1
      else
        echo \"OK \${topics[\$i]}\"
      fi
    done
    exit \"\$rc\"
  " > "$run/validation/preflight_messages.txt" 2>&1
  local rc=$?
  set -e
  return "$rc"
}

recorder_supports_start_paused()
{
  docker_ros "ros2 bag record --help 2>&1 | grep -q -- '--start-paused'"
}

wait_for_recorder_subscriptions()
{
  local run=$1
  local deadline=$((SECONDS + 60))

  while (( SECONDS < deadline )); do
    if grep -Fq "All requested topics are subscribed" \
      "$run/logs/recorder.log" 2>/dev/null; then
      return 0
    fi

    if [[ -s "$run/runtime/recorder.exit_code" ]]; then
      return 1
    fi
    sleep 0.25
  done
  return 1
}

resume_recorder()
{
  local run=$1
  local fifo="$run/runtime/recorder.stdin"
  local deadline=$((SECONDS + 10))

  while (( SECONDS < deadline )); do
    if [[ -p "$fifo" ]]; then
      break
    fi

    if [[ -s "$run/runtime/recorder.exit_code" ]]; then
      return 1
    fi

    sleep 0.1
  done

  [[ -p "$fifo" ]] || return 1

  # rosbag2 Humble keyboard control uses SPACE to toggle paused state.
  if ! timeout 2s bash -c 'printf " " > "$1"' _ "$fifo"; then
    return 1
  fi

  # Confirm that rosbag2 resumed before recording the run.
  deadline=$((SECONDS + 5))
  while (( SECONDS < deadline )); do
    if grep -Fq "Resuming recording." "$run/logs/recorder.log" 2>/dev/null ||
       grep -Fq "Recording..." "$run/logs/recorder.log" 2>/dev/null; then
      return 0
    fi

    if [[ -s "$run/runtime/recorder.exit_code" ]]; then
      return 1
    fi

    sleep 0.1
  done

  return 1
}

stop_recorder_gracefully()
{
  local run=$1
  local grace=${2:-35}
  local fifo="$run/runtime/recorder.stdin"
  local pgid_file="$run/runtime/recorder.pgid"

  [[ -s "$pgid_file" ]] || return 0

  local pgid
  pgid=$(<"$pgid_file")

  if ! docker exec "$CONTAINER" bash -lc \
    "kill -0 -- -'$pgid' 2>/dev/null"; then
    return 0
  fi

  if [[ ! -p "$fifo" ]]; then
    log "Recorder control FIFO missing; falling back to process-group stop"
    stop_component "$run" recorder "$grace"
    return $?
  fi

  log "Sending Ctrl-C through recorder PTY for graceful rosbag2 shutdown"

  if ! timeout 2s bash -c 'printf "\003" > "$1"' _ "$fifo"; then
    log "Failed to send Ctrl-C through recorder PTY"
    stop_component "$run" recorder "$grace"
    return 1
  fi

  local deadline=$((SECONDS + grace))

  while (( SECONDS < deadline )); do
    if ! docker exec "$CONTAINER" bash -lc \
      "kill -0 -- -'$pgid' 2>/dev/null"; then
      return 0
    fi
    sleep 0.25
  done

  log "Recorder did not exit after PTY Ctrl-C; falling back to process-group stop"
  stop_component "$run" recorder 10
}

start_stack()
{
  require_environment

  if active_run >/dev/null 2>&1; then
    die "An active run already exists: $(active_run). Run '$0 stop' first."
  fi

  log "Starting Docker container: $CONTAINER"
  docker start "$CONTAINER" >/dev/null

  local leftovers
  leftovers=$(known_processes)
  if [[ -n "$leftovers" ]]; then
    echo "$leftovers" >&2
    die "Residual sensor processes exist. Run '$0 cleanup' once, then retry."
  fi

  local label=${1:-run}

  log "Starting PTP target: $PTP_TARGET"
  sudo systemctl start "$PTP_TARGET"

  for _ in {1..15}; do
    if systemctl is-active --quiet mid360-ptp4l.service \
      && systemctl is-active --quiet mid360-phc2sys.service; then
      break
    fi
    sleep 1
  done

  systemctl is-active --quiet mid360-ptp4l.service \
    || die "mid360-ptp4l.service did not become active"
  systemctl is-active --quiet mid360-phc2sys.service \
    || die "mid360-phc2sys.service did not become active"

  local run
  run=$(create_run "$label")
  log "Run directory: $run"

  systemctl status "$PTP_TARGET" --no-pager \
    > "$run/logs/ptp_status_at_start.txt" 2>&1 || true
  journalctl -u mid360-ptp4l.service -u mid360-phc2sys.service -n 200 --no-pager \
    > "$run/logs/ptp_recent.log" 2>&1 || true

  docker exec -u 0 "$CONTAINER" bash -lc '
    chgrp video /dev/nvpps0
    chmod 0660 /dev/nvpps0
  '

  log "Starting NvPPS publisher"
  launch_component "$run" nvpps
  sleep 2

  log "Starting Livox MID-360"
  launch_component "$run" livox
  sleep 2

  log "Starting four integrated JPEG cameras"
  launch_component "$run" cameras

  if ! wait_for_topics "$run" "$BASE_TOPIC_COUNT"; then
    log "Base 11-topic gate failed"
    status_stack || true
    stop_stack || true
    die "Sensor stack did not publish all required topics"
  fi

  log "All base 11 topics are present. Warm-up: ${WARMUP_SECONDS}s"
  sleep "$WARMUP_SECONDS"

  if [[ "$READOUT_DELAY_ENABLED" == "true" ]]; then
    log "Applying C2 per-camera ReadOutDelay after camera initialization"

    if ! "$READOUT_DELAY_SCRIPT" "$run"; then
      log "ReadOutDelay application failed"
      stop_stack || true
      die "Could not apply or verify C2 ReadOutDelay"
    fi
  else
    log "C2 per-camera ReadOutDelay is disabled"
  fi

  log "Preparing I2C devices for exposure samplers"

  docker exec -u 0 "$CONTAINER" bash -lc '
    for dev in       /dev/i2c-9       /dev/i2c-10       /dev/i2c-11       /dev/i2c-12
    do
      [[ -e "$dev" ]] || {
        echo "Missing I2C device: $dev" >&2
        exit 2
      }

      chmod 0666 "$dev"
    done
  '

  if ! docker_ros     "ros2 pkg prefix c2_exposure_sampler >/dev/null 2>&1"; then
    stop_stack || true
    die "c2_exposure_sampler package is unavailable"
  fi

  log "Starting four frame-triggered C2 exposure samplers"
  launch_component "$run" exposure

  if ! wait_for_topics "$run"; then
    log "Full 15-topic gate failed"
    status_stack || true
    stop_stack || true
    die "Exposure samplers did not publish all required topics"
  fi

  log "All 15 required topics are present"

  touch "$run/runtime/STARTED"
  log "Stack is ready"
  echo "$run"
}

stop_ptp_stack()
{
  log "Stopping PTP target and child services"

  # Stopping the target does not guarantee its required services stop.
  # Shut down ptp4l and phc2sys explicitly.
  sudo systemctl stop \
    "$PTP_TARGET" \
    mid360-phc2sys.service \
    mid360-ptp4l.service \
    || true

  local deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    if ! systemctl is-active --quiet mid360-phc2sys.service \
      && ! systemctl is-active --quiet mid360-ptp4l.service; then
      return 0
    fi
    sleep 0.25
  done

  # Retry once if either PTP service remains active.
  sudo systemctl stop mid360-phc2sys.service mid360-ptp4l.service || true
}

write_shutdown_status()
{
  local run=$1
  local residual
  residual=$(known_processes)

  local target_state
  local ptp4l_state
  local phc2sys_state
  target_state=$(systemctl is-active "$PTP_TARGET" 2>/dev/null || true)
  ptp4l_state=$(systemctl is-active mid360-ptp4l.service 2>/dev/null || true)
  phc2sys_state=$(systemctl is-active mid360-phc2sys.service 2>/dev/null || true)

  local clean=true
  [[ -z "$residual" ]] || clean=false
  [[ "$ptp4l_state" != "active" ]] || clean=false
  [[ "$phc2sys_state" != "active" ]] || clean=false

  {
    echo "checked_at: \"$(date --iso-8601=seconds)\""
    echo "ptp_target_state: $target_state"
    echo "ptp4l_state: $ptp4l_state"
    echo "phc2sys_state: $phc2sys_state"

    if [[ "$clean" == "true" ]]; then
      echo "shutdown_clean: true"
      echo "residual_processes: []"
      touch "$run/runtime/STOPPED_CLEAN"
      rm -f "$run/runtime/STOPPED_DIRTY"
    else
      echo "shutdown_clean: false"
      if [[ -z "$residual" ]]; then
        echo "residual_processes: []"
      else
        echo "residual_processes: |"
        while IFS= read -r line; do
          printf '  %s\n' "$line"
        done <<<"$residual"
      fi
      touch "$run/runtime/STOPPED_DIRTY"
      rm -f "$run/runtime/STOPPED_CLEAN"
    fi
  } > "$run/validation/runtime_shutdown.yaml"
}

stop_stack()
{
  require_environment
  local run=""
  run=$(active_run 2>/dev/null || true)

  if [[ -n "$run" ]] && docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
    log "Stopping rosbag recorder"
    stop_component "$run" recorder 35

    log "Stopping exposure sampler process group"
    stop_component "$run" exposure 12

    log "Stopping camera process group"
    stop_component "$run" cameras 12

    log "Stopping NvPPS process group"
    stop_component "$run" nvpps 8

    log "Stopping Livox process group"
    stop_component "$run" livox 12

    local residual
    residual=$(known_processes)
    if [[ -n "$residual" ]]; then
      log "Residual processes detected; applying exact-name cleanup"
      cleanup_known_processes
    fi

    known_processes > "$run/validation/processes_after_stop.txt" 2>&1 || true
  fi

  stop_ptp_stack

  if [[ -n "$run" ]]; then
    {
      systemctl status "$PTP_TARGET" --no-pager || true
      systemctl status mid360-ptp4l.service --no-pager || true
      systemctl status mid360-phc2sys.service --no-pager || true
    } > "$run/logs/ptp_status_at_stop.txt" 2>&1
    df -h "$DATA_ROOT" > "$run/config/storage_after.txt" 2>&1 || true
    write_shutdown_status "$run"
  fi

  rm -f "$ACTIVE_FILE"
  log "Stack stopped"
}

cleanup_stack()
{
  require_environment
  log "Starting Docker container for cleanup: $CONTAINER"
  docker start "$CONTAINER" >/dev/null

  local run=""
  run=$(active_run 2>/dev/null || true)

  if [[ -n "$run" ]]; then
    for name in recorder exposure cameras nvpps livox; do
      stop_component "$run" "$name" 8 || true
    done
  fi

  cleanup_known_processes

  stop_ptp_stack

  local residual
  residual=$(known_processes)
  if [[ -n "$residual" ]]; then
    echo "$residual" >&2
    die "Cleanup could not remove every process"
  fi

  if [[ -n "$run" ]]; then
    write_shutdown_status "$run"
  fi

  rm -f "$ACTIVE_FILE"
  log "Cleanup complete; no known sensor or recorder process remains"
}

status_stack()
{
  require_environment

  echo "Recording stack status"
  echo "PTP target    : $(systemctl is-active "$PTP_TARGET" 2>/dev/null || true)"
  echo "ptp4l         : $(systemctl is-active mid360-ptp4l.service 2>/dev/null || true)"
  echo "phc2sys       : $(systemctl is-active mid360-phc2sys.service 2>/dev/null || true)"
  echo "Docker        : $(docker inspect "$CONTAINER" --format '{{.State.Status}}' 2>/dev/null || echo missing)"

  local run=""
  run=$(active_run 2>/dev/null || true)
  echo "Active run    : ${run:-none}"

  if [[ -n "$run" ]] && docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
    echo
    echo "Process groups:"
    for name in nvpps livox cameras exposure recorder; do
      if group_alive "$run" "$name"; then
        echo "  $name: RUNNING pgid=$(<"$run/runtime/${name}.pgid")"
      elif [[ -s "$run/runtime/${name}.pgid" ]]; then
        echo "  $name: STOPPED pgid=$(<"$run/runtime/${name}.pgid")"
      else
        echo "  $name: NOT_STARTED"
      fi
    done

    echo
    echo "Required topics:"
    local listing
    listing=$(docker_ros "ros2 topic list -t" 2>/dev/null || true)
    for i in "${!TOPICS[@]}"; do
      if grep -Fq "${TOPICS[$i]} [${EXPECTED_TYPES[$i]}]" <<<"$listing"; then
        echo "  [OK]   ${TOPICS[$i]}"
      else
        echo "  [MISS] ${TOPICS[$i]}"
      fi
    done
  fi

  echo
  echo "Known residual processes:"
  local residual
  residual=$(known_processes)
  if [[ -n "$residual" ]]; then
    echo "$residual"
  else
    echo "  none"
  fi

  echo
  df -h "$DATA_ROOT" | tail -1
}

record_dataset()
{
  require_environment

  local duration=${1:-}
  local label=${2:-recording}

  [[ "$duration" =~ ^[0-9]+$ ]] && (( duration > 0 )) \
    || die "Duration must be a positive integer number of seconds"

  local auto_started=0
  local run=""

  run=$(active_run 2>/dev/null || true)
  if [[ -z "$run" ]]; then
    auto_started=1
    start_stack "$label"
    run=$(active_run)
  fi

  [[ ! -e "$run/bag/metadata.yaml" ]] \
    || die "This run already contains a recording: $run/bag"

  log "Preflight: receiving all required topics in parallel"
  if ! parallel_message_preflight "$run"; then
    [[ "$auto_started" -eq 1 ]] && stop_stack || true
    die "At least one required topic did not provide a message"
  fi

  local actual_duration=$((duration + ANCHOR_GUARD_SECONDS * 2))

  if ! recorder_supports_start_paused; then
    [[ "$auto_started" -eq 1 ]] && stop_stack || true
    die "rosbag2 does not support --start-paused"
  fi

  # Start paused; resume only after all 15 topic subscriptions are ready.
  local start_paused=true

  cat > "$run/config/recording_request.yaml" <<EOF
created_at: "$(date --iso-8601=seconds)"
purpose: "$(sanitize_label "$label")"
requested_duration_sec: $duration
actual_record_duration_sec: $actual_duration
anchor_guard_before_after_sec: $ANCHOR_GUARD_SECONDS
recorder_started_paused: $start_paused
recorder_mode: start_paused_resume_after_all_subscriptions
recorder_settle_after_subscriptions_sec: $RECORDER_SETTLE_SECONDS
fsync_enabled: $FSYNC_ENABLED
fsync:
  enabled: $FSYNC_ENABLED
  source: "$FSYNC_SOURCE"
  generator: "$FSYNC_GENERATOR"
  frequency_hz: $FSYNC_FREQUENCY_HZ
  duty_cycle_percent: $FSYNC_DUTY_CYCLE_PERCENT
  trigger_mode: $FSYNC_TRIGGER_MODE
  fsync_mfp: $FSYNC_MFP
readout_delay:
  enabled: $READOUT_DELAY_ENABLED
  apply_stage: after_camera_topic_gate_and_warmup
  snapshot: config/camera_readout_delay.yaml
  validation_log: validation/readout_delay_apply.txt
exposure:
  enabled: true
  source: gw5300_current_shutter
  trigger: camera_timestamp_debug
  status_read_bytes: 101
  raw_count_unit_ns: 12500
  expected_rate_hz: 30
  frame_association_status: provisional
  topics:
    - /camera/cam0/exposure_debug
    - /camera/cam1/exposure_debug
    - /camera/cam2/exposure_debug
    - /camera/cam3/exposure_debug
camera_format: jpeg
jpeg_quality: 95
resolution: [1920, 1240]
EOF

  log "Launching MCAP recorder in paused mode"
  launch_component \
    "$run" recorder \
    "$start_paused" "$run/bag" \
    "${TOPICS[@]}"

  if ! wait_for_recorder_subscriptions "$run"; then
    stop_component "$run" recorder 15 || true
    [[ "$auto_started" -eq 1 ]] && stop_stack || true
    die "Recorder did not subscribe to all requested topics"
  fi

  log "All recorder subscriptions are ready"
  log "Recorder is paused; settling for ${RECORDER_SETTLE_SECONDS}s"
  sleep "$RECORDER_SETTLE_SECONDS"

  log "Resuming MCAP recorder"
  if ! resume_recorder "$run"; then
    stop_component "$run" recorder 15 || true
    [[ "$auto_started" -eq 1 ]] && stop_stack || true
    die "Failed to resume paused rosbag2 recorder"
  fi

  date --iso-8601=ns > "$run/runtime/recording_window_start.txt"
  log "Recording ${duration}s (including ${ANCHOR_GUARD_SECONDS}s guards at both ends: ${actual_duration}s)"
  sleep "$actual_duration"
  date --iso-8601=ns > "$run/runtime/recording_window_stop.txt"

  log "Stopping recorder and flushing MCAP"
  stop_recorder_gracefully "$run" 35

  sleep 3
  sync

  set +e
  docker_ros "ros2 bag info -s mcap '$run/bag'" \
    > "$run/validation/bag_info.txt" 2>&1
  local info_rc=$?

  docker_ros "python3 '$FINALIZER' --dataset '$run'" \
    > >(tee "$run/validation/finalizer_stdout.txt") \
    2> >(tee "$run/validation/finalizer_stderr.txt" >&2)
  local finalize_rc=$?
  set -e

  if [[ "$info_rc" -ne 0 ]]; then
    log "ros2 bag info failed"
    finalize_rc=2
  fi

  if [[ "$auto_started" -eq 1 ]]; then
    stop_stack
  fi

  echo
  echo "Dataset: $run"
  if [[ "$finalize_rc" -eq 0 ]]; then
    echo "Result : READY"
    echo "Input  : $run/manifest.yaml"
  else
    echo "Result : NOT_READY"
    echo "Check  : $run/validation/result.txt"
  fi

  return "$finalize_rc"
}

usage()
{
  cat <<EOF
Usage:
  $0 start [label]
  $0 status
  $0 record <seconds> [label]
  $0 stop
  $0 cleanup

Recommended one-command recording:
  $0 record 60 driving_test

The recorder starts paused. The script waits until all 15 recorder
subscriptions are ready, settles briefly, resumes the recorder, and then marks
the recording window. This prevents early-subscribed topics from accumulating
extra startup messages. Dataset validation still uses the common usable sensor
interval. Every component runs in its own Linux process group.
EOF
}

main()
{
  local command=${1:-}
  case "$command" in
    start)
      shift
      start_stack "${1:-run}"
      ;;
    status)
      status_stack
      ;;
    record)
      shift
      record_dataset "${1:-}" "${2:-recording}"
      ;;
    stop)
      stop_stack
      ;;
    cleanup)
      cleanup_stack
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
