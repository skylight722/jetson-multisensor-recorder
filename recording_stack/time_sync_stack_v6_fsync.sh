#!/usr/bin/env bash
set -Eeuo pipefail

# Host-side FSYNC control for the V6 recorder.

STACK="${STACK:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"
BASE_STACK="${BASE_STACK:-$STACK/time_sync_stack_v6_core.sh}"

TSC_TRIGGER="${TSC_TRIGGER:-$HOME/fsync_tools/tsc_trigger}"
TSC_DEVICE="${TSC_DEVICE:-/dev/cdi_tsc}"

FSYNC_STATE_DIR="${FSYNC_STATE_DIR:-$STACK/runtime_fsync}"
FSYNC_PID_FILE="$FSYNC_STATE_DIR/tsc_trigger.pid"
FSYNC_LOG_FILE="$FSYNC_STATE_DIR/tsc_trigger.log"
FSYNC_INFO_FILE="$FSYNC_STATE_DIR/fsync_runtime.env"

TRIGGER_MODE_EXPECTED=5
FSYNC_MFP_EXPECTED=1

DT_TSC_STATUS="/proc/device-tree/tsc_sig_gen@c6a0000/status"
DT_GENERATOR_STATUS="/proc/device-tree/tsc_sig_gen@c6a0000/generator@480/status"

mkdir -p "$FSYNC_STATE_DIR"


timestamp()
{
  date --iso-8601=seconds
}


log()
{
  printf '[%s] %s\n' "$(timestamp)" "$*"
}


die()
{
  log "ERROR: $*" >&2
  exit 1
}


read_text_dt_property()
{
  local path="$1"

  if [[ ! -r "$path" ]]; then
    return 1
  fi

  tr -d '\0' < "$path"
}


read_module_parameter()
{
  local module="$1"
  local parameter="$2"
  local path="/sys/module/$module/parameters/$parameter"

  if [[ ! -r "$path" ]]; then
    return 1
  fi

  cat "$path"
}


get_saved_tsc_pid()
{
  if [[ ! -r "$FSYNC_PID_FILE" ]]; then
    return 1
  fi

  local pid
  pid="$(cat "$FSYNC_PID_FILE" 2>/dev/null || true)"

  if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
    return 1
  fi

  printf '%s\n' "$pid"
}


tsc_is_running()
{
  local pid
  pid="$(get_saved_tsc_pid)" || return 1

  if ! kill -0 "$pid" 2>/dev/null; then
    return 1
  fi

  local expected_exe
  local actual_exe

  expected_exe="$(readlink -f "$TSC_TRIGGER")"
  actual_exe="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"

  [[ -n "$actual_exe" && "$actual_exe" == "$expected_exe" ]]
}


remove_stale_pid_file()
{
  if [[ -e "$FSYNC_PID_FILE" ]] && ! tsc_is_running; then
    log "Removing stale TSC PID file."
    rm -f "$FSYNC_PID_FILE"
  fi
}


fsync_precheck()
{
  local failures=0

  echo "FSYNC checks"

  printf '%-36s : ' "Base stack"
  if [[ -x "$BASE_STACK" ]]; then
    echo "PASS ($BASE_STACK)"
  else
    echo "FAIL ($BASE_STACK)"
    failures=$((failures + 1))
  fi

  printf '%-36s : ' "TSC trigger executable"
  if [[ -x "$TSC_TRIGGER" ]]; then
    echo "PASS ($TSC_TRIGGER)"
  else
    echo "FAIL ($TSC_TRIGGER)"
    failures=$((failures + 1))
  fi

  printf '%-36s : ' "TSC character device"
  if [[ -c "$TSC_DEVICE" ]]; then
    echo "PASS ($TSC_DEVICE)"
  else
    echo "FAIL ($TSC_DEVICE)"
    failures=$((failures + 1))
  fi

  printf '%-36s : ' "TSC device read/write permission"
  if [[ -r "$TSC_DEVICE" && -w "$TSC_DEVICE" ]]; then
    echo "PASS"
  else
    echo "FAIL"
    failures=$((failures + 1))
  fi

  printf '%-36s : ' "video group membership"
  if id -nG | tr ' ' '\n' | grep -qx video; then
    echo "PASS"
  else
    echo "FAIL"
    failures=$((failures + 1))
  fi

  local trigger_mode
  trigger_mode="$(
    read_module_parameter tier4_imx490 trigger_mode 2>/dev/null ||
      true
  )"

  printf '%-36s : ' "tier4_imx490 trigger_mode"
  if [[ "$trigger_mode" == "$TRIGGER_MODE_EXPECTED" ]]; then
    echo "PASS ($trigger_mode)"
  else
    echo "FAIL (actual=${trigger_mode:-missing}, expected=5)"
    failures=$((failures + 1))
  fi

  local fsync_mfp
  fsync_mfp="$(
    read_module_parameter tier4_imx490 fsync_mfp 2>/dev/null ||
      true
  )"

  printf '%-36s : ' "tier4_imx490 fsync_mfp"
  if [[ "$fsync_mfp" == "$FSYNC_MFP_EXPECTED" ]]; then
    echo "PASS ($fsync_mfp)"
  else
    echo "FAIL (actual=${fsync_mfp:-missing}, expected=1)"
    failures=$((failures + 1))
  fi

  local max9296_module
  max9296_module="$(modinfo -n tier4_max9296 2>/dev/null || true)"

  printf '%-36s : ' "MAX9296 MFP1 patch"
  if [[ -f "$max9296_module" ]] &&
     grep -aFq 'Failed to disable LOCK output on MFP1.' "$max9296_module"; then
    echo "PASS"
  else
    echo "FAIL"
    failures=$((failures + 1))
  fi

  local tsc_status
  tsc_status="$(read_text_dt_property "$DT_TSC_STATUS" || true)"

  printf '%-36s : ' "TSC device-tree status"
  if [[ "$tsc_status" == "okay" ]]; then
    echo "PASS ($tsc_status)"
  else
    echo "FAIL (${tsc_status:-missing})"
    failures=$((failures + 1))
  fi

  local generator_status
  generator_status="$(
    read_text_dt_property "$DT_GENERATOR_STATUS" ||
      true
  )"

  printf '%-36s : ' "TSC generator@480 status"
  if [[ "$generator_status" == "okay" ]]; then
    echo "PASS ($generator_status)"
  else
    echo "FAIL (${generator_status:-missing})"
    failures=$((failures + 1))
  fi

  remove_stale_pid_file

  printf '%-36s : ' "Existing TSC trigger process"
  if tsc_is_running; then
    echo "RUNNING (pid=$(get_saved_tsc_pid))"
  else
    echo "NOT RUNNING"
  fi

  echo

  if (( failures > 0 )); then
    echo "FSYNC PRECHECK: FAIL ($failures item(s))"
    return 1
  fi

  echo "FSYNC PRECHECK: PASS"
}


start_tsc()
{
  fsync_precheck

  remove_stale_pid_file

  if tsc_is_running; then
    log "TSC FSYNC is already running. pid=$(get_saved_tsc_pid)"
    return 0
  fi

  {
    echo
    echo "FSYNC started at: $(timestamp)"
    echo "Trigger binary: $TSC_TRIGGER"
    echo "TSC device: $TSC_DEVICE"
    echo "Frequency: 30 Hz"
    echo "Duty cycle: 30 percent"
    echo "Generator: TSC_EDGE_OUT2 / generator@480"
    echo "Camera mode: trigger_mode=5 / fsync_mfp=1"
  } >> "$FSYNC_LOG_FILE"

  log "Starting Host TSC 30 Hz FSYNC."

  setsid "$TSC_TRIGGER" \
    </dev/null \
    >>"$FSYNC_LOG_FILE" \
    2>&1 &

  local pid=$!

  printf '%s\n' "$pid" > "${FSYNC_PID_FILE}.tmp"
  mv "${FSYNC_PID_FILE}.tmp" "$FSYNC_PID_FILE"

  sleep 1

  if ! tsc_is_running; then
    rm -f "$FSYNC_PID_FILE"

    echo "TSC trigger failed to remain running." >&2
    echo "--- TSC log ---" >&2
    tail -n 100 "$FSYNC_LOG_FILE" >&2 || true

    return 1
  fi

  cat > "$FSYNC_INFO_FILE" <<INFO
FSYNC_ENABLED=true
FSYNC_SOURCE=TSC_EDGE_OUT2
FSYNC_GENERATOR=generator@480
FSYNC_FREQUENCY_HZ=30
FSYNC_DUTY_CYCLE_PERCENT=30
FSYNC_TRIGGER_MODE=5
FSYNC_MFP=1
FSYNC_TSC_PID=$pid
FSYNC_STARTED_AT=$(timestamp)
INFO

  log "TSC FSYNC started successfully. pid=$pid"
}


stop_tsc()
{
  remove_stale_pid_file

  if ! tsc_is_running; then
    log "TSC FSYNC is not running."
    rm -f "$FSYNC_PID_FILE"
    return 0
  fi

  local pid
  pid="$(get_saved_tsc_pid)"

  log "Stopping Host TSC FSYNC. pid=$pid"

  kill -TERM "$pid" 2>/dev/null || true

  local attempt

  for attempt in $(seq 1 50); do
    if ! kill -0 "$pid" 2>/dev/null; then
      break
    fi

    sleep 0.1
  done

  if kill -0 "$pid" 2>/dev/null; then
    log "TSC process did not stop after SIGTERM; sending SIGKILL."
    kill -KILL "$pid" 2>/dev/null || true
    sleep 0.2
  fi

  rm -f "$FSYNC_PID_FILE"

  {
    echo "FSYNC stopped at: $(timestamp)"
  } >> "$FSYNC_LOG_FILE"

  log "TSC FSYNC stopped."
}


show_fsync_status()
{
  echo
  echo "FSYNC status"

  if tsc_is_running; then
    echo "state       : running"
    echo "pid         : $(get_saved_tsc_pid)"
  else
    echo "state       : stopped"
  fi

  echo "trigger     : $TSC_TRIGGER"
  echo "device      : $TSC_DEVICE"
  echo "frequency   : 30 Hz"
  echo "duty cycle  : 30 percent"
  echo "source      : TSC_EDGE_OUT2 / generator@480"
  echo "trigger mode: $(read_module_parameter tier4_imx490 trigger_mode 2>/dev/null || echo missing)"
  echo "fsync mfp   : $(read_module_parameter tier4_imx490 fsync_mfp 2>/dev/null || echo missing)"
  echo "log         : $FSYNC_LOG_FILE"
}


run_base_stack()
{
  env \
    FSYNC_ENABLED=true \
    FSYNC_SOURCE=TSC_EDGE_OUT2 \
    FSYNC_GENERATOR=generator@480 \
    FSYNC_FREQUENCY_HZ=30 \
    FSYNC_DUTY_CYCLE_PERCENT=30 \
    FSYNC_TRIGGER_MODE=5 \
    FSYNC_MFP=1 \
    FSYNC_RUNTIME_INFO="$FSYNC_INFO_FILE" \
    "$BASE_STACK" "$@"
}


record_command()
{
  start_tsc

  local base_rc=0
  local stop_rc=0

  record_cleanup()
  {
    local original_rc=$?

    trap - EXIT INT TERM

    stop_tsc || true

    exit "$original_rc"
  }

  trap record_cleanup EXIT INT TERM

  run_base_stack record "$@" || base_rc=$?

  stop_tsc || stop_rc=$?

  trap - EXIT INT TERM

  if (( base_rc != 0 )); then
    return "$base_rc"
  fi

  return "$stop_rc"
}


start_command()
{
  start_tsc

  local base_rc=0

  run_base_stack start "$@" || base_rc=$?

  if (( base_rc != 0 )); then
    log "Base stack start failed. Stopping FSYNC."
    stop_tsc || true

    return "$base_rc"
  fi
}


stop_command()
{
  local base_rc=0
  local fsync_rc=0

  # Stop camera capture before disabling the FSYNC trigger.
  run_base_stack stop "$@" || base_rc=$?
  stop_tsc || fsync_rc=$?

  if (( base_rc != 0 )); then
    return "$base_rc"
  fi

  return "$fsync_rc"
}


cleanup_command()
{
  local base_rc=0
  local fsync_rc=0

  run_base_stack cleanup "$@" || base_rc=$?
  stop_tsc || fsync_rc=$?

  if (( base_rc != 0 )); then
    return "$base_rc"
  fi

  return "$fsync_rc"
}


usage()
{
  cat <<USAGE
Usage:
  $0 precheck
  $0 fsync-start
  $0 fsync-stop
  $0 start [label]
  $0 record <seconds> <label>
  $0 status
  $0 stop
  $0 cleanup

Examples:
  $0 precheck
  $0 record 60 straight_driving_fsync
  $0 start manual_fsync_test
  $0 status
  $0 stop
USAGE
}


main()
{
  local command="${1:-}"
  shift || true

  case "$command" in
    precheck)
      fsync_precheck
      ;;

    fsync-start)
      start_tsc
      ;;

    fsync-stop)
      stop_tsc
      ;;

    start)
      start_command "$@"
      ;;

    record)
      if (( $# < 2 )); then
        die "record requires <seconds> and <label>."
      fi

      record_command "$@"
      ;;

    status)
      base_status_rc=0

      run_base_stack status "$@" || base_status_rc=$?
      show_fsync_status

      return "$base_status_rc"
      ;;

    stop)
      stop_command "$@"
      ;;

    cleanup)
      cleanup_command "$@"
      ;;

    help|-h|--help|"")
      usage
      ;;

    *)
      usage
      die "Unknown command: $command"
      ;;
  esac
}


main "$@"
