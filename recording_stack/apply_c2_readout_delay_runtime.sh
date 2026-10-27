#!/usr/bin/env bash
set -Eeuo pipefail

RUN=${1:?Dataset run directory is required}

STACK_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CTRL=${T4CAM_CTRL:-/usr/bin/t4cam-ctrl}
I2C_CONFIG=${T4CAM_I2C_CONFIG:-"$STACK_DIR/config/t4cam_C2_i2c.yaml"}

LOG="$RUN/validation/readout_delay_apply.txt"
SNAPSHOT="$RUN/config/camera_readout_delay.yaml"
CONFIG_COPY="$RUN/config/t4cam_C2_i2c.yaml"

mkdir -p "$RUN/validation" "$RUN/config" "$RUN/runtime"

exec > >(tee "$LOG") 2>&1

rollback()
{
  local original_rc=$?

  trap - ERR INT TERM HUP
  set +e

  echo
  echo "ReadOutDelay: FAILED"
  echo "Restoring representative camera ports to 0.0 ms."

  for port in 1 3 5 7
  do
    sudo "$CTRL" \
      "$I2C_CONFIG" \
      "$port" \
      set read-out-delay 0.0 \
      >/dev/null 2>&1 || true
  done

  touch "$RUN/runtime/READOUT_DELAY_FAILED"

  return "$original_rc"
}

trap rollback ERR INT TERM HUP

[[ -x "$CTRL" ]]
[[ -f "$I2C_CONFIG" ]]
[[ -d "$RUN" ]]

cp -a "$I2C_CONFIG" "$CONFIG_COPY"

echo "Applying camera ReadOutDelay"
echo "Applied at : $(date --iso-8601=ns)"
echo "Tool       : $CTRL"
echo "I2C config : $I2C_CONFIG"
echo "Run        : $RUN"
echo
echo "Requested configuration:"
echo "  cam0 Left  / port 1 / i2c-9  : 11.321 ms"
echo "  cam1 Front / port 3 / i2c-10 :  0.00 ms"
echo "  cam2 Right / port 5 / i2c-11 : 25.000 ms"
echo "  cam3 Rear  / port 7 / i2c-12 : 16.673 ms"

echo
echo "Setting camera delays"

sudo "$CTRL" "$I2C_CONFIG" 1 set read-out-delay 11.321
sudo "$CTRL" "$I2C_CONFIG" 3 set read-out-delay 0.0
sudo "$CTRL" "$I2C_CONFIG" 5 set read-out-delay 25.000
sudo "$CTRL" "$I2C_CONFIG" 7 set read-out-delay 16.673

sleep 1

READBACK_VALUE=""

read_port()
{
  local port=$1
  local camera=$2
  local output
  local value

  echo
  echo "$camera / port $port"

  output=$(sudo "$CTRL" \
    "$I2C_CONFIG" \
    "$port" \
    get read-out-delay)

  printf '%s\n' "$output"

  value=$(
    sed -n \
      's/.*Read Out Delay:[[:space:]]*//p' \
      <<<"$output" |
      tail -n 1
  )

  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]]

  READBACK_VALUE=$value
}

numeric_equal()
{
  local actual=$1
  local expected=$2

  awk \
    -v actual="$actual" \
    -v expected="$expected" \
    'BEGIN { exit !((actual + 0) == (expected + 0)) }'
}

echo
echo "Reading back camera delays"

read_port 1 "cam0 Left"
LEFT_READBACK=$READBACK_VALUE

read_port 3 "cam1 Front"
FRONT_READBACK=$READBACK_VALUE

read_port 5 "cam2 Right"
RIGHT_READBACK=$READBACK_VALUE

read_port 7 "cam3 Rear"
REAR_READBACK=$READBACK_VALUE

echo
echo "Checking readback values"

numeric_equal "$LEFT_READBACK" 11
echo "cam0 Left  : PASS ($LEFT_READBACK)"

numeric_equal "$FRONT_READBACK" 0
echo "cam1 Front : PASS ($FRONT_READBACK)"

numeric_equal "$RIGHT_READBACK" 25
echo "cam2 Right : PASS ($RIGHT_READBACK)"

numeric_equal "$REAR_READBACK" 16
echo "cam3 Rear  : PASS ($REAR_READBACK)"

SDK_VERSION=$(
  dpkg-query \
    -W \
    -f='${Version}' \
    t4cam_sdk \
    2>/dev/null ||
    true
)

I2C_SHA256=$(
  sha256sum "$I2C_CONFIG" |
    awk '{print $1}'
)

cat > "$SNAPSHOT" <<EOF
applied_at: "$(date --iso-8601=ns)"
enabled: true
status: applied_and_read_back
apply_stage: after_camera_topic_gate_and_warmup
tool:
  executable: "$CTRL"
  package: t4cam_sdk
  package_version: "$SDK_VERSION"
i2c_config:
  source: "$I2C_CONFIG"
  dataset_snapshot: config/t4cam_C2_i2c.yaml
  sha256: "$I2C_SHA256"
fsync:
  frequency_hz: 30
  period_ms: 33.333333
precision:
  requested_unit: ms
  get_display_behavior: integer_only_observed
  fractional_register_application_status: verified_by_raw_v4l2_timestamp_phase_validation

phase_calibration:
  enabled: true
  status: verified
  reference_clock_domain: raw_v4l2_timestamp_ns
  reference_camera: cam1
  reference_direction: front

  target_phase_ms:
    cam0_left: 8.330
    cam1_front: 0.000
    cam2_right: 25.000
    cam3_rear: 16.660

  calibrated_read_out_delay_ms:
    cam0_left: 11.321
    cam1_front: 0.000
    cam2_right: 25.000
    cam3_rear: 16.673

  reference_validation:
    date: 2026-08-07
    dataset: run_20260807_234839_final_phase_auto_smoke
    measured_period_ms: 33.329
    measured_phase_ms:
      cam0_left: 8.333951
      cam1_front: 0.000000
      cam2_right: 24.999236
      cam3_rear: 16.660724
    phase_error_us:
      cam0_left: 3.951
      cam1_front: 0.000
      cam2_right: -0.764
      cam3_rear: 0.724
    maximum_absolute_phase_error_us: 3.951
    result: PASS

  per_run_phase_validation: not_performed_by_apply_script

cameras:
  cam0:
    direction: left
    i2c_device: /dev/i2c-9
    representative_port: 1
    alias_port: 2
    requested_read_out_delay_ms: 11.321
    readback_display_ms: $LEFT_READBACK
  cam1:
    direction: front
    i2c_device: /dev/i2c-10
    representative_port: 3
    alias_port: 4
    requested_read_out_delay_ms: 0.0
    readback_display_ms: $FRONT_READBACK
  cam2:
    direction: right
    i2c_device: /dev/i2c-11
    representative_port: 5
    alias_port: 6
    requested_read_out_delay_ms: 25.000
    readback_display_ms: $RIGHT_READBACK
  cam3:
    direction: rear
    i2c_device: /dev/i2c-12
    representative_port: 7
    alias_port: 8
    requested_read_out_delay_ms: 16.673
    readback_display_ms: $REAR_READBACK
verification:
  raw_v4l2_timestamp_phase_status: verified
  baseline_phase_compensation_status: verified
  calibrated_target_phase_status: verified
  per_run_register_readback_status: verified_by_integer_display
  per_run_raw_timestamp_phase_status: not_measured
  optical_exposure_phase_status: unverified
  exposure_reference_status: unverified
EOF

touch "$RUN/runtime/READOUT_DELAY_APPLIED"
rm -f "$RUN/runtime/READOUT_DELAY_FAILED"

trap - ERR INT TERM HUP

echo
echo "ReadOutDelay: PASS"
echo "Snapshot : $SNAPSHOT"
echo "Log      : $LOG"
