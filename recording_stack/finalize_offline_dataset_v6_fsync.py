#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import shutil
import statistics
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CALIBRATION_BUNDLE_ROOT = Path(
    os.environ.get(
        "CAMERA_CALIBRATION_BUNDLE_ROOT",
        str(Path.home() / "shared_dir/camera_calibration/final_4cam"),
    )
)

CAMERAS = {
    "cam0": {
        "direction": "left",
        "frame_id": "camera_left",
        "intrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "intrinsics/cam0_left.yaml"
        ),
        "extrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "extrinsics/cam0_left.yaml"
        ),
    },
    "cam1": {
        "direction": "front",
        "frame_id": "camera_front",
        "intrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "intrinsics/cam1_front.yaml"
        ),
        "extrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "extrinsics/cam1_front.yaml"
        ),
    },
    "cam2": {
        "direction": "right",
        "frame_id": "camera_right",
        "intrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "intrinsics/cam2_right.yaml"
        ),
        "extrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "extrinsics/cam2_right.yaml"
        ),
    },
    "cam3": {
        "direction": "rear",
        "frame_id": "camera_rear",
        "intrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "intrinsics/cam3_rear.yaml"
        ),
        "extrinsic_source": str(
            CALIBRATION_BUNDLE_ROOT / "extrinsics/cam3_rear.yaml"
        ),
    },
}

JPEG_TOPICS = {
    cam: f"/camera/{cam}/image_raw/compressed" for cam in CAMERAS
}
DEBUG_TOPICS = {
    cam: f"/camera/{cam}/timestamp_debug" for cam in CAMERAS
}
EXPOSURE_TOPICS = {
    cam: f"/camera/{cam}/exposure_debug" for cam in CAMERAS
}

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
NVPPS_TOPIC = "/time_sync/nvpps"

EXPOSURE_TYPE = "c2_exposure_sampler/msg/CameraExposureDebug"
EXPOSURE_RETRY_RATIO_MAX = 0.05
EXPOSURE_QUEUE_WARN_DEPTH = 5
EXPOSURE_JOIN_RATIO_MIN = 0.995
EXPOSURE_RAW_COUNT_UNIT_NS = 12_500

REQUIRED_TYPES: Dict[str, str] = {
    **{topic: "sensor_msgs/msg/CompressedImage" for topic in JPEG_TOPICS.values()},
    **{topic: "time_sync_msgs/msg/CameraTimestampDebug" for topic in DEBUG_TOPICS.values()},
    **{topic: EXPOSURE_TYPE for topic in EXPOSURE_TOPICS.values()},
    LIDAR_TOPIC: "livox_ros_driver2/msg/CustomMsg",
    IMU_TOPIC: "sensor_msgs/msg/Imu",
    NVPPS_TOPIC: "time_sync_msgs/msg/NvPpsCorrelation",
}


def stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def rate_hz(stamps: List[int]) -> float:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return (len(stamps) - 1) * 1_000_000_000.0 / (stamps[-1] - stamps[0])


def non_increasing(values: List[int]) -> int:
    return sum(1 for a, b in zip(values, values[1:]) if b <= a)


def estimated_missing_from_sequence(values: List[int]) -> int:
    missing = 0
    for a, b in zip(values, values[1:]):
        if b > a + 1:
            missing += b - a - 1
    return missing


def interpolate_tsc_to_ptp(
    tsc_ns: int,
    anchor_tsc: List[int],
    anchor_ptp: List[int],
) -> Optional[int]:
    if len(anchor_tsc) < 2 or len(anchor_tsc) != len(anchor_ptp):
        return None

    pairs = sorted(zip(anchor_tsc, anchor_ptp))
    xs = [item[0] for item in pairs]
    ys = [item[1] for item in pairs]

    if tsc_ns < xs[0] or tsc_ns > xs[-1]:
        return None
    if tsc_ns == xs[-1]:
        return ys[-1]

    index = bisect.bisect_right(xs, tsc_ns) - 1
    index = max(0, min(index, len(xs) - 2))
    x0, x1 = xs[index], xs[index + 1]
    y0, y1 = ys[index], ys[index + 1]

    if x1 <= x0:
        return None

    ratio = (tsc_ns - x0) / float(x1 - x0)
    return int(round(y0 + ratio * (y1 - y0)))


def parse_camera_driver_deltas(
    log_path: Path,
    bag_start_ns: Optional[int],
    bag_end_ns: Optional[int],
) -> Dict[str, Optional[float]]:
    metrics = {
        "queue_overflow": [],
        "encode_failures": [],
        "sequence_missing": [],
        "sequence_non_increasing": [],
        "timestamp_non_increasing": [],
        "capture_fps": [],
        "publish_fps": [],
        "avg_encode_publish_ms": [],
    }

    if not log_path.is_file() or bag_start_ns is None or bag_end_ns is None:
        return {
            "queue_overflow_delta_during_bag": None,
            "encode_failures_delta_during_bag": None,
            "sequence_missing_delta_during_bag": None,
            "sequence_non_increasing_delta_during_bag": None,
            "timestamp_non_increasing_delta_during_bag": None,
            "capture_fps_near_end": None,
            "publish_fps_near_end": None,
            "avg_encode_publish_ms_near_end": None,
        }

    line_pattern = re.compile(r"\[([0-9]+\.[0-9]+)\]")
    value_patterns = {
        key: re.compile(rf"\b{re.escape(key)}=([0-9]+(?:\.[0-9]+)?)")
        for key in metrics
    }

    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        stamp_match = line_pattern.search(line)
        if not stamp_match:
            continue
        stamp_ns = int(float(stamp_match.group(1)) * 1_000_000_000)

        for key, pattern in value_patterns.items():
            match = pattern.search(line)
            if match:
                metrics[key].append((stamp_ns, float(match.group(1))))

    def cumulative_delta(key: str) -> Optional[int]:
        samples = metrics[key]
        before = [value for stamp, value in samples if stamp <= bag_start_ns]
        during = [value for stamp, value in samples if stamp <= bag_end_ns]
        if not before or not during:
            return None
        return int(round(during[-1] - before[-1]))

    def near_end(key: str) -> Optional[float]:
        samples = [value for stamp, value in metrics[key] if stamp <= bag_end_ns]
        return samples[-1] if samples else None

    return {
        "queue_overflow_delta_during_bag": cumulative_delta("queue_overflow"),
        "encode_failures_delta_during_bag": cumulative_delta("encode_failures"),
        "sequence_missing_delta_during_bag": cumulative_delta("sequence_missing"),
        "sequence_non_increasing_delta_during_bag": cumulative_delta("sequence_non_increasing"),
        "timestamp_non_increasing_delta_during_bag": cumulative_delta("timestamp_non_increasing"),
        "capture_fps_near_end": near_end("capture_fps"),
        "publish_fps_near_end": near_end("publish_fps"),
        "avg_encode_publish_ms_near_end": near_end("avg_encode_publish_ms"),
    }


def jpeg_dimensions(data: bytes) -> Optional[Tuple[int, int]]:
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return None

    index = 2
    sof_markers = {
        0xC0, 0xC1, 0xC2, 0xC3,
        0xC5, 0xC6, 0xC7,
        0xC9, 0xCA, 0xCB,
        0xCD, 0xCE, 0xCF,
    }

    while index + 3 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue

        while index < len(data) and data[index] == 0xFF:
            index += 1

        if index >= len(data):
            return None

        marker = data[index]
        index += 1

        if marker in {0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            continue

        if index + 1 >= len(data):
            return None

        length = (data[index] << 8) + data[index + 1]
        if length < 2 or index + length > len(data):
            return None

        if marker in sof_markers:
            if length < 7:
                return None
            height = (data[index + 3] << 8) + data[index + 4]
            width = (data[index + 5] << 8) + data[index + 6]
            return width, height

        index += length

    return None


@dataclass
class CameraData:
    image_headers: List[int] = field(default_factory=list)
    jpeg_sizes: List[int] = field(default_factory=list)
    bad_jpeg_format: int = 0
    bad_jpeg_dimensions: int = 0

    debug_headers: List[int] = field(default_factory=list)
    raw_tsc: List[int] = field(default_factory=list)
    sequences: List[int] = field(default_factory=list)
    buffer_errors: int = 0
    not_enqueued: int = 0
    publish_not_requested: int = 0



@dataclass
class ExposureData:
    header_stamps: List[int] = field(default_factory=list)

    sample_sequences: List[int] = field(default_factory=list)
    frame_sequences: List[int] = field(default_factory=list)
    frame_raw_tsc: List[int] = field(default_factory=list)
    frame_header_stamps: List[int] = field(default_factory=list)

    queue_depths: List[int] = field(default_factory=list)
    dqbuf_to_sample_start_ns: List[int] = field(default_factory=list)

    raw_counts: List[int] = field(default_factory=list)
    exposure_times_ns: List[int] = field(default_factory=list)
    valid_flags: List[bool] = field(default_factory=list)
    attempts: List[int] = field(default_factory=list)
    error_codes: List[int] = field(default_factory=list)

    sample_before_ns: List[int] = field(default_factory=list)
    sample_after_ns: List[int] = field(default_factory=list)

    camera_name_mismatches: int = 0


def load_request(dataset: Path) -> Dict[str, Any]:
    path = dataset / "config" / "recording_request.yaml"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_yaml_dict(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {}

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    return data if isinstance(data, dict) else {}


def as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default

    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
        "enabled",
    }


def copy_calibration(dataset: Path) -> Dict[str, Any]:
    calibration_root = dataset / "calibration"
    destination = calibration_root / "intrinsics"
    destination.mkdir(parents=True, exist_ok=True)

    bundle_manifest_source = (
        CALIBRATION_BUNDLE_ROOT / "CALIBRATION_MANIFEST.yaml"
    )
    bundle_manifest_target = (
        calibration_root / "CALIBRATION_MANIFEST.yaml"
    )

    if bundle_manifest_source.is_file():
        shutil.copy2(
            bundle_manifest_source,
            bundle_manifest_target,
        )

    result: Dict[str, Any] = {}

    for cam, info in CAMERAS.items():
        source = Path(info["intrinsic_source"])
        target = (
            destination
            / f"{cam}_{info['direction']}_intrinsics.yaml"
        )

        if source.is_file():
            shutil.copy2(source, target)
            result[cam] = {
                "status": "available",
                "path": str(target.relative_to(dataset)),
                "source": str(source),
            }
        else:
            result[cam] = {
                "status": "missing",
                "path": None,
                "source": str(source),
            }

    return result


def copy_extrinsics(dataset: Path) -> Dict[str, Any]:
    destination = dataset / "calibration" / "extrinsics"
    destination.mkdir(parents=True, exist_ok=True)

    required_fields = (
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
    )

    result: Dict[str, Any] = {}

    for cam, info in CAMERAS.items():
        source = Path(info["extrinsic_source"])
        target = (
            destination
            / f"{cam}_{info['direction']}_extrinsics.yaml"
        )

        if not source.is_file():
            result[cam] = {
                "status": "missing",
                "path": None,
                "source": str(source),
                "parent_frame": info["frame_id"],
                "child_frame": "lidar_frame",
            }
            continue

        try:
            payload = yaml.safe_load(
                source.read_text(encoding="utf-8")
            )

            transform = payload[
                info["frame_id"]
            ]["lidar_frame"]

            for field_name in required_fields:
                float(transform[field_name])

        except Exception as exc:
            result[cam] = {
                "status": "invalid",
                "path": None,
                "source": str(source),
                "parent_frame": info["frame_id"],
                "child_frame": "lidar_frame",
                "error": str(exc),
            }
            continue

        shutil.copy2(source, target)

        result[cam] = {
            "status": "available",
            "path": str(target.relative_to(dataset)),
            "source": str(source),
            "parent_frame": info["frame_id"],
            "child_frame": "lidar_frame",
        }

    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--bag", default="bag")
    parser.add_argument("--minimum-anchor-coverage", type=float, default=0.85)
    args = parser.parse_args()

    dataset = Path(args.dataset).expanduser().resolve()
    bag = (dataset / args.bag).resolve()
    validation_dir = dataset / "validation"
    validation_dir.mkdir(parents=True, exist_ok=True)

    ready = dataset / "READY"
    not_ready = dataset / "NOT_READY"
    ready.unlink(missing_ok=True)
    not_ready.unlink(missing_ok=True)

    request = load_request(dataset)

    fsync_request = request.get("fsync") or {}
    if not isinstance(fsync_request, dict):
        fsync_request = {}

    fsync_enabled_raw = fsync_request.get(
        "enabled",
        request.get("fsync_enabled", False),
    )

    if isinstance(fsync_enabled_raw, bool):
        fsync_enabled = fsync_enabled_raw
    else:
        fsync_enabled = (
            str(fsync_enabled_raw).strip().lower()
            in {"1", "true", "yes", "on"}
        )

    fsync_runtime_snapshot = (
        dataset / "config" / "fsync_runtime.env"
    )

    fsync_config = {
        "enabled": fsync_enabled,
        "source": fsync_request.get("source"),
        "generator": fsync_request.get("generator"),
        "frequency_hz": fsync_request.get("frequency_hz"),
        "duty_cycle_percent": fsync_request.get(
            "duty_cycle_percent"
        ),
        "trigger_mode": fsync_request.get("trigger_mode"),
        "fsync_mfp": fsync_request.get("fsync_mfp"),
        "runtime_snapshot": (
            "config/fsync_runtime.env"
            if fsync_runtime_snapshot.is_file()
            else None
        ),
    }

    readout_request = request.get("readout_delay") or {}
    if not isinstance(readout_request, dict):
        readout_request = {}

    readout_enabled = as_bool(
        readout_request.get("enabled"),
        default=False,
    )

    readout_snapshot_rel = str(
        readout_request.get(
            "snapshot",
            "config/camera_readout_delay.yaml",
        )
    )
    readout_validation_log_rel = str(
        readout_request.get(
            "validation_log",
            "validation/readout_delay_apply.txt",
        )
    )

    readout_snapshot_path = Path(readout_snapshot_rel)
    if not readout_snapshot_path.is_absolute():
        readout_snapshot_path = dataset / readout_snapshot_path

    readout_validation_log_path = Path(readout_validation_log_rel)
    if not readout_validation_log_path.is_absolute():
        readout_validation_log_path = (
            dataset / readout_validation_log_path
        )

    readout_applied_marker = (
        dataset / "runtime" / "READOUT_DELAY_APPLIED"
    )
    readout_failed_marker = (
        dataset / "runtime" / "READOUT_DELAY_FAILED"
    )

    readout_snapshot_exists = readout_snapshot_path.is_file()
    readout_validation_log_exists = (
        readout_validation_log_path.is_file()
    )
    readout_applied_marker_exists = (
        readout_applied_marker.is_file()
    )
    readout_failed_marker_exists = (
        readout_failed_marker.exists()
    )

    readout_snapshot = load_yaml_dict(readout_snapshot_path)
    readout_status = readout_snapshot.get("status")
    readout_snapshot_enabled = as_bool(
        readout_snapshot.get("enabled"),
        default=False,
    )

    readout_precision = readout_snapshot.get("precision") or {}
    if not isinstance(readout_precision, dict):
        readout_precision = {}

    readout_verification = (
        readout_snapshot.get("verification") or {}
    )
    if not isinstance(readout_verification, dict):
        readout_verification = {}

    readout_cameras = readout_snapshot.get("cameras") or {}
    if not isinstance(readout_cameras, dict):
        readout_cameras = {}

    readout_issues: List[str] = []

    if readout_enabled:
        if not readout_snapshot_exists:
            readout_issues.append(
                f"Missing ReadOutDelay snapshot: "
                f"{readout_snapshot_rel}"
            )

        if not readout_validation_log_exists:
            readout_issues.append(
                f"Missing ReadOutDelay validation log: "
                f"{readout_validation_log_rel}"
            )

        if not readout_applied_marker_exists:
            readout_issues.append(
                "Missing runtime/READOUT_DELAY_APPLIED marker"
            )

        if readout_failed_marker_exists:
            readout_issues.append(
                "runtime/READOUT_DELAY_FAILED marker exists"
            )

        if not readout_snapshot_enabled:
            readout_issues.append(
                "ReadOutDelay snapshot does not report enabled=true"
            )

        if readout_status != "applied_and_read_back":
            readout_issues.append(
                "ReadOutDelay snapshot status is not "
                "'applied_and_read_back'"
            )

        requested_apply_stage = readout_request.get("apply_stage")
        snapshot_apply_stage = readout_snapshot.get("apply_stage")

        if (
            requested_apply_stage
            and snapshot_apply_stage
            and requested_apply_stage != snapshot_apply_stage
        ):
            readout_issues.append(
                "ReadOutDelay apply-stage mismatch between "
                "recording request and snapshot"
            )

        for cam, info in CAMERAS.items():
            camera_delay = readout_cameras.get(cam)

            if not isinstance(camera_delay, dict):
                readout_issues.append(
                    f"Missing ReadOutDelay camera entry: {cam}"
                )
                continue

            if "requested_read_out_delay_ms" not in camera_delay:
                readout_issues.append(
                    f"{cam}: missing requested_read_out_delay_ms"
                )

            if "readback_display_ms" not in camera_delay:
                readout_issues.append(
                    f"{cam}: missing readback_display_ms"
                )

            snapshot_direction = camera_delay.get("direction")
            if (
                snapshot_direction is not None
                and snapshot_direction != info["direction"]
            ):
                readout_issues.append(
                    f"{cam}: direction mismatch "
                    f"({snapshot_direction} != {info['direction']})"
                )

    readout_ready = (
        True
        if not readout_enabled
        else len(readout_issues) == 0
    )

    readout_result = (
        "DISABLED"
        if not readout_enabled
        else "PASS"
        if readout_ready
        else "CHECK"
    )

    readout_config = {
        "enabled": readout_enabled,
        "ready": readout_ready,
        "result": readout_result,
        "status": (
            "disabled"
            if not readout_enabled
            else readout_status
        ),
        "apply_stage": readout_snapshot.get(
            "apply_stage",
            readout_request.get("apply_stage"),
        ),
        "snapshot": readout_snapshot_rel,
        "snapshot_exists": readout_snapshot_exists,
        "validation_log": readout_validation_log_rel,
        "validation_log_exists": readout_validation_log_exists,
        "runtime_applied_marker": (
            "runtime/READOUT_DELAY_APPLIED"
        ),
        "runtime_applied_marker_exists": (
            readout_applied_marker_exists
        ),
        "runtime_failed_marker_exists": (
            readout_failed_marker_exists
        ),
        "tool": readout_snapshot.get("tool") or {},
        "i2c_config": (
            readout_snapshot.get("i2c_config") or {}
        ),
        "precision": readout_precision,
        "cameras": readout_cameras,
        "verification": readout_verification,
        "issues": readout_issues,
    }

    calibration = copy_calibration(dataset)
    extrinsics = copy_extrinsics(dataset)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    actual_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }

    type_errors = {
        topic: {"expected": expected, "actual": actual_types.get(topic)}
        for topic, expected in REQUIRED_TYPES.items()
        if actual_types.get(topic) != expected
    }

    if type_errors:
        result = {
            "status": "NOT_READY",
            "reason": "Required topic/type mismatch",
            "type_errors": type_errors,
        }
        (validation_dir / "validation.yaml").write_text(
            yaml.safe_dump(result, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        not_ready.touch()
        print(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
        return 2

    classes = {
        topic: get_message(message_type)
        for topic, message_type in REQUIRED_TYPES.items()
    }

    cameras = {cam: CameraData() for cam in CAMERAS}
    exposures = {cam: ExposureData() for cam in CAMERAS}

    topic_to_camera = {
        **{topic: ("jpeg", cam) for cam, topic in JPEG_TOPICS.items()},
        **{topic: ("debug", cam) for cam, topic in DEBUG_TOPICS.items()},
        **{topic: ("exposure", cam) for cam, topic in EXPOSURE_TOPICS.items()},
    }

    lidar_timebase: List[int] = []
    lidar_scan_end: List[int] = []
    lidar_headers: List[int] = []
    lidar_empty = 0
    lidar_point_mismatch = 0

    imu_headers: List[int] = []

    nvpps_headers: List[int] = []
    nvpps_tsc: List[int] = []
    nvpps_ptp: List[int] = []
    nvpps_events: List[int] = []

    bag_record_min: Optional[int] = None
    bag_record_max: Optional[int] = None

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic not in REQUIRED_TYPES:
            continue

        msg = deserialize_message(serialized, classes[topic])
        record_ns = int(record_ns)
        bag_record_min = record_ns if bag_record_min is None else min(bag_record_min, record_ns)
        bag_record_max = record_ns if bag_record_max is None else max(bag_record_max, record_ns)

        camera_target = topic_to_camera.get(topic)
        if camera_target:
            kind, cam = camera_target
            data = cameras[cam]

            if kind == "jpeg":
                header = stamp_ns(msg.header)
                data.image_headers.append(header)
                payload = bytes(msg.data)
                data.jpeg_sizes.append(len(payload))
                if "jpeg" not in str(msg.format).lower():
                    data.bad_jpeg_format += 1
                if jpeg_dimensions(payload) != (1920, 1240):
                    data.bad_jpeg_dimensions += 1

            elif kind == "debug":
                current_header = int(msg.current_header_stamp_ns)
                data.debug_headers.append(current_header)
                data.raw_tsc.append(int(msg.raw_v4l2_timestamp_ns))
                data.sequences.append(int(msg.sequence))
                data.buffer_errors += int(bool(msg.v4l2_buffer_error))
                data.not_enqueued += int(not bool(msg.image_enqueued))
                data.publish_not_requested += int(not bool(msg.publish_requested))

            elif kind == "exposure":
                exp_data = exposures[cam]

                exp_data.header_stamps.append(
                    stamp_ns(msg.header)
                )
                exp_data.sample_sequences.append(
                    int(msg.sample_sequence)
                )
                exp_data.frame_sequences.append(
                    int(msg.frame_sequence)
                )
                exp_data.frame_raw_tsc.append(
                    int(msg.frame_raw_v4l2_timestamp_ns)
                )
                exp_data.frame_header_stamps.append(
                    int(msg.frame_header_stamp_ns)
                )

                exp_data.queue_depths.append(
                    int(msg.queue_depth_before_read)
                )
                exp_data.dqbuf_to_sample_start_ns.append(
                    int(msg.dqbuf_to_sample_start_ns)
                )

                exp_data.raw_counts.append(
                    int(msg.raw_count)
                )
                exp_data.exposure_times_ns.append(
                    int(msg.exposure_time_ns)
                )
                exp_data.valid_flags.append(
                    bool(msg.valid)
                )
                exp_data.attempts.append(
                    int(msg.attempts)
                )
                exp_data.error_codes.append(
                    int(msg.error_code)
                )

                exp_data.sample_before_ns.append(
                    int(msg.sample_realtime_before_ns)
                )
                exp_data.sample_after_ns.append(
                    int(msg.sample_realtime_after_ns)
                )

                exp_data.camera_name_mismatches += int(
                    str(msg.camera_name) != cam
                )


        elif topic == LIDAR_TOPIC:
            lidar_headers.append(stamp_ns(msg.header))
            lidar_timebase.append(int(msg.timebase))
            actual_points = len(msg.points)
            if actual_points == 0:
                lidar_empty += 1
                lidar_scan_end.append(int(msg.timebase))
            else:
                lidar_scan_end.append(
                    int(msg.timebase) + max(int(point.offset_time) for point in msg.points)
                )
            if int(msg.point_num) != actual_points:
                lidar_point_mismatch += 1

        elif topic == IMU_TOPIC:
            imu_headers.append(stamp_ns(msg.header))

        elif topic == NVPPS_TOPIC:
            nvpps_headers.append(stamp_ns(msg.header))
            nvpps_tsc.append(int(msg.tsc_ns))
            nvpps_ptp.append(int(msg.ptp_ns))
            nvpps_events.append(int(msg.event_number))

    results: Dict[str, Any] = {}
    overall_pass = True
    requested_duration_sec = float(request.get("requested_duration_sec") or 0.0)
    usable_camera_ranges: Dict[str, Tuple[int, int]] = {}

    for cam, info in CAMERAS.items():
        data = cameras[cam]
        exp_data = exposures[cam]

        image_count = len(data.image_headers)
        debug_count = len(data.debug_headers)

        image_counter = Counter(data.image_headers)
        debug_counter = Counter(data.debug_headers)
        matched = sum((image_counter & debug_counter).values())
        pair_denominator = max(image_count, debug_count, 1)
        pair_ratio = matched / pair_denominator

        exposure_count = len(exp_data.frame_sequences)
        exposure_rate = rate_hz(exp_data.frame_header_stamps)

        exposure_sample_missing = estimated_missing_from_sequence(
            exp_data.sample_sequences
        )
        exposure_frame_missing = estimated_missing_from_sequence(
            exp_data.frame_sequences
        )

        retry_count = sum(
            attempts > 1 for attempts in exp_data.attempts
        )
        retry_ratio = (
            retry_count / exposure_count
            if exposure_count > 0
            else 1.0
        )

        backlog_events = sum(
            depth > EXPOSURE_QUEUE_WARN_DEPTH
            for depth in exp_data.queue_depths
        )

        valid_count = sum(exp_data.valid_flags)
        invalid_count = exposure_count - valid_count

        conversion_mismatches = sum(
            1
            for valid, raw_count, exposure_time_ns
            in zip(
                exp_data.valid_flags,
                exp_data.raw_counts,
                exp_data.exposure_times_ns,
            )
            if (
                valid
                and exposure_time_ns
                != raw_count * EXPOSURE_RAW_COUNT_UNIT_NS
            )
        )

        nonpositive_valid_payload = sum(
            1
            for valid, raw_count, exposure_time_ns
            in zip(
                exp_data.valid_flags,
                exp_data.raw_counts,
                exp_data.exposure_times_ns,
            )
            if (
                valid
                and (
                    raw_count <= 0
                    or exposure_time_ns <= 0
                )
            )
        )

        valid_error_count = sum(
            1
            for valid, error_code
            in zip(
                exp_data.valid_flags,
                exp_data.error_codes,
            )
            if valid and error_code != 0
        )

        invalid_nonzero_payload = sum(
            1
            for valid, raw_count, exposure_time_ns
            in zip(
                exp_data.valid_flags,
                exp_data.raw_counts,
                exp_data.exposure_times_ns,
            )
            if (
                not valid
                and (
                    raw_count != 0
                    or exposure_time_ns != 0
                )
            )
        )

        attempts_out_of_range = sum(
            attempts < 1 or attempts > 3
            for attempts in exp_data.attempts
        )

        debug_by_sequence = {
            int(sequence): (
                int(raw_tsc),
                int(header_stamp),
            )
            for sequence, raw_tsc, header_stamp
            in zip(
                data.sequences,
                data.raw_tsc,
                data.debug_headers,
            )
        }

        matched_exposure_debug = 0
        exposure_debug_metadata_mismatches = 0

        for (
            frame_sequence,
            frame_raw_tsc,
            frame_header_stamp,
        ) in zip(
            exp_data.frame_sequences,
            exp_data.frame_raw_tsc,
            exp_data.frame_header_stamps,
        ):
            debug_item = debug_by_sequence.get(
                int(frame_sequence)
            )

            if debug_item is None:
                continue

            matched_exposure_debug += 1

            debug_raw_tsc, debug_header_stamp = debug_item

            if (
                frame_raw_tsc != debug_raw_tsc
                or frame_header_stamp != debug_header_stamp
            ):
                exposure_debug_metadata_mismatches += 1

        exposure_debug_join_ratio = (
            matched_exposure_debug / exposure_count
            if exposure_count > 0
            else 0.0
        )

        exposure_sequence_set = set(
            exp_data.frame_sequences
        )
        debug_sequence_set = set(
            data.sequences
        )

        exposure_first_sequence = (
            min(exp_data.frame_sequences)
            if exp_data.frame_sequences
            else None
        )
        exposure_last_sequence = (
            max(exp_data.frame_sequences)
            if exp_data.frame_sequences
            else None
        )
        debug_first_sequence = (
            min(data.sequences)
            if data.sequences
            else None
        )
        debug_last_sequence = (
            max(data.sequences)
            if data.sequences
            else None
        )

        edge_unmatched_exposure_before = 0
        edge_unmatched_exposure_after = 0
        internal_unmatched_exposure = 0
        internal_unmatched_debug = 0
        overlap_join_ratio = 0.0

        if (
            exposure_first_sequence is not None
            and exposure_last_sequence is not None
            and debug_first_sequence is not None
            and debug_last_sequence is not None
        ):
            overlap_first_sequence = max(
                exposure_first_sequence,
                debug_first_sequence,
            )
            overlap_last_sequence = min(
                exposure_last_sequence,
                debug_last_sequence,
            )

            unmatched_exposure_sequences = (
                exposure_sequence_set - debug_sequence_set
            )

            edge_unmatched_exposure_before = sum(
                sequence < debug_first_sequence
                for sequence in unmatched_exposure_sequences
            )

            edge_unmatched_exposure_after = sum(
                sequence > debug_last_sequence
                for sequence in unmatched_exposure_sequences
            )

            internal_unmatched_exposure = sum(
                overlap_first_sequence
                <= sequence
                <= overlap_last_sequence
                for sequence in unmatched_exposure_sequences
            )

            unmatched_debug_sequences = (
                debug_sequence_set - exposure_sequence_set
            )

            internal_unmatched_debug = sum(
                overlap_first_sequence
                <= sequence
                <= overlap_last_sequence
                for sequence in unmatched_debug_sequences
            )

            exposure_overlap_set = {
                sequence
                for sequence in exposure_sequence_set
                if (
                    overlap_first_sequence
                    <= sequence
                    <= overlap_last_sequence
                )
            }

            debug_overlap_set = {
                sequence
                for sequence in debug_sequence_set
                if (
                    overlap_first_sequence
                    <= sequence
                    <= overlap_last_sequence
                )
            }

            overlap_union = (
                exposure_overlap_set | debug_overlap_set
            )
            overlap_intersection = (
                exposure_overlap_set & debug_overlap_set
            )

            overlap_join_ratio = (
                len(overlap_intersection)
                / len(overlap_union)
                if overlap_union
                else 0.0
            )
        else:
            overlap_first_sequence = None
            overlap_last_sequence = None

        internal_debug_coverage_ratio = overlap_join_ratio

        sample_read_durations_ms = [
            (after_ns - before_ns) / 1_000_000.0
            for before_ns, after_ns
            in zip(
                exp_data.sample_before_ns,
                exp_data.sample_after_ns,
            )
            if after_ns >= before_ns
        ]

        exposure_pass = (
            exposure_count > 0
            and 27.0 <= exposure_rate <= 31.5
            and non_increasing(
                exp_data.sample_sequences
            ) == 0
            and exposure_sample_missing == 0
            and non_increasing(
                exp_data.frame_sequences
            ) == 0
            and exposure_frame_missing == 0
            and non_increasing(
                exp_data.frame_raw_tsc
            ) == 0
            and non_increasing(
                exp_data.frame_header_stamps
            ) == 0
            and invalid_count == 0
            and conversion_mismatches == 0
            and nonpositive_valid_payload == 0
            and valid_error_count == 0
            and invalid_nonzero_payload == 0
            and attempts_out_of_range == 0
            and retry_ratio <= EXPOSURE_RETRY_RATIO_MAX
            and backlog_events == 0
            and exp_data.camera_name_mismatches == 0
            # Raw whole-bag join ratio is diagnostic only.
            # Topic publication latency can create unmatched records
            # at recorder start/stop boundaries.
            and overlap_join_ratio >= EXPOSURE_JOIN_RATIO_MIN
            and internal_unmatched_exposure == 0
            and internal_unmatched_debug == 0
            and exposure_debug_metadata_mismatches == 0
        )

        covered_values: List[int] = []
        edge_before = 0
        edge_after = 0
        covered_duration_sec = 0.0

        if nvpps_tsc and data.raw_tsc:
            anchor_first = min(nvpps_tsc)
            anchor_last = max(nvpps_tsc)
            covered_values = [
                value for value in data.raw_tsc
                if anchor_first <= value <= anchor_last
            ]
            covered = len(covered_values)
            coverage_ratio = covered / len(data.raw_tsc)
            edge_before = sum(value < anchor_first for value in data.raw_tsc)
            edge_after = sum(value > anchor_last for value in data.raw_tsc)
            if len(covered_values) >= 2:
                covered_duration_sec = (
                    covered_values[-1] - covered_values[0]
                ) / 1_000_000_000.0
                usable_camera_ranges[cam] = (
                    covered_values[0],
                    covered_values[-1],
                )
        else:
            covered = 0
            coverage_ratio = 0.0

        driver = parse_camera_driver_deltas(
            dataset / "logs" / "cameras" / f"{cam}.log",
            bag_record_min,
            bag_record_max,
        )
        driver_deltas_ok = all(
            driver[key] in (None, 0)
            for key in (
                "queue_overflow_delta_during_bag",
                "encode_failures_delta_during_bag",
                "sequence_missing_delta_during_bag",
                "sequence_non_increasing_delta_during_bag",
                "timestamp_non_increasing_delta_during_bag",
            )
        )
        usable_duration_ok = (
            requested_duration_sec <= 0.0
            or covered_duration_sec >= max(0.0, requested_duration_sec - 0.25)
        )

        camera_pass = (
            image_count > 0
            and debug_count > 0
            and 27.0 <= rate_hz(data.image_headers) <= 31.5
            and 27.0 <= rate_hz(data.debug_headers) <= 31.5
            and non_increasing(data.image_headers) == 0
            and non_increasing(data.debug_headers) == 0
            and non_increasing(data.raw_tsc) == 0
            and non_increasing(data.sequences) == 0
            and estimated_missing_from_sequence(data.sequences) == 0
            and data.bad_jpeg_format == 0
            and data.bad_jpeg_dimensions == 0
            and data.buffer_errors == 0
            and data.not_enqueued == 0
            and data.publish_not_requested == 0
            and pair_ratio >= 0.995
            and coverage_ratio >= args.minimum_anchor_coverage
            and usable_duration_ok
            and driver_deltas_ok
            and exposure_pass
        )

        results[cam] = {
            "direction": info["direction"],
            "image_count": image_count,
            "image_rate_hz": round(rate_hz(data.image_headers), 6),
            "debug_count": debug_count,
            "debug_rate_hz": round(rate_hz(data.debug_headers), 6),
            "image_debug_matched": matched,
            "image_debug_pair_ratio": round(pair_ratio, 9),
            "sequence_missing": estimated_missing_from_sequence(data.sequences),
            "sequence_non_increasing": non_increasing(data.sequences),
            "raw_tsc_non_increasing": non_increasing(data.raw_tsc),
            "jpeg_bad_format": data.bad_jpeg_format,
            "jpeg_bad_dimensions": data.bad_jpeg_dimensions,
            "buffer_errors": data.buffer_errors,
            "not_enqueued": data.not_enqueued,
            "publish_not_requested": data.publish_not_requested,
            "anchor_covered_frames": covered,
            "anchor_coverage_ratio": round(coverage_ratio, 9),
            "edge_frames_before_first_anchor": edge_before,
            "edge_frames_after_last_anchor": edge_after,
            "anchor_covered_duration_sec": round(covered_duration_sec, 6),
            "requested_duration_satisfied": usable_duration_ok,
            "driver_runtime": driver,
            "average_jpeg_kib": round(statistics.mean(data.jpeg_sizes) / 1024.0, 3)
                if data.jpeg_sizes else 0.0,
            "exposure": {
                "topic": EXPOSURE_TOPICS[cam],
                "message_type": EXPOSURE_TYPE,
                "count": exposure_count,
                "rate_hz": round(exposure_rate, 6),
                "valid_count": valid_count,
                "invalid_count": invalid_count,
                "sample_sequence_missing": exposure_sample_missing,
                "sample_sequence_non_increasing": non_increasing(
                    exp_data.sample_sequences
                ),
                "frame_sequence_first": exposure_first_sequence,
                "frame_sequence_last": exposure_last_sequence,
                "frame_sequence_missing": exposure_frame_missing,
                "frame_sequence_non_increasing": non_increasing(
                    exp_data.frame_sequences
                ),
                "frame_raw_tsc_non_increasing": non_increasing(
                    exp_data.frame_raw_tsc
                ),
                "frame_header_non_increasing": non_increasing(
                    exp_data.frame_header_stamps
                ),
                "raw_count_min": (
                    min(exp_data.raw_counts)
                    if exp_data.raw_counts
                    else None
                ),
                "raw_count_max": (
                    max(exp_data.raw_counts)
                    if exp_data.raw_counts
                    else None
                ),
                "exposure_time_ms_mean": (
                    round(
                        statistics.mean(
                            exp_data.exposure_times_ns
                        ) / 1_000_000.0,
                        6,
                    )
                    if exp_data.exposure_times_ns
                    else None
                ),
                "exposure_time_ms_min": (
                    round(
                        min(exp_data.exposure_times_ns)
                        / 1_000_000.0,
                        6,
                    )
                    if exp_data.exposure_times_ns
                    else None
                ),
                "exposure_time_ms_max": (
                    round(
                        max(exp_data.exposure_times_ns)
                        / 1_000_000.0,
                        6,
                    )
                    if exp_data.exposure_times_ns
                    else None
                ),
                "raw_count_to_time_mismatches": conversion_mismatches,
                "nonpositive_valid_payload": nonpositive_valid_payload,
                "valid_error_count": valid_error_count,
                "invalid_nonzero_payload": invalid_nonzero_payload,
                "attempts_out_of_range": attempts_out_of_range,
                "retry_count": retry_count,
                "retry_ratio": round(retry_ratio, 6),
                "retry_ratio_limit": EXPOSURE_RETRY_RATIO_MAX,
                "max_queue_depth": (
                    max(exp_data.queue_depths)
                    if exp_data.queue_depths
                    else None
                ),
                "backlog_events": backlog_events,
                "queue_warn_depth": EXPOSURE_QUEUE_WARN_DEPTH,
                "timestamp_debug_joined": matched_exposure_debug,
                "timestamp_debug_join_ratio": round(
                    exposure_debug_join_ratio,
                    9,
                ),
                "debug_sequence_first": debug_first_sequence,
                "debug_sequence_last": debug_last_sequence,
                "overlap_sequence_first": overlap_first_sequence,
                "overlap_sequence_last": overlap_last_sequence,
                "edge_unmatched_exposure_before": (
                    edge_unmatched_exposure_before
                ),
                "edge_unmatched_exposure_after": (
                    edge_unmatched_exposure_after
                ),
                "internal_unmatched_exposure": (
                    internal_unmatched_exposure
                ),
                "internal_unmatched_debug": (
                    internal_unmatched_debug
                ),
                "overlap_join_ratio": round(
                    overlap_join_ratio,
                    9,
                ),
                "internal_debug_coverage_ratio": round(
                    internal_debug_coverage_ratio,
                    9,
                ),
                "frame_metadata_mismatches": (
                    exposure_debug_metadata_mismatches
                ),
                "camera_name_mismatches": (
                    exp_data.camera_name_mismatches
                ),
                "sample_read_ms_mean": (
                    round(
                        statistics.mean(
                            sample_read_durations_ms
                        ),
                        6,
                    )
                    if sample_read_durations_ms
                    else None
                ),
                "sample_read_ms_max": (
                    round(
                        max(sample_read_durations_ms),
                        6,
                    )
                    if sample_read_durations_ms
                    else None
                ),
                "trigger_frame_association": "verified",
                "optical_application_frame_status": "unverified",
                "result": "PASS" if exposure_pass else "CHECK",
            },
            "result": "PASS" if camera_pass else "CHECK",
        }
        overall_pass = overall_pass and camera_pass

    all_exposure_pass = all(
        results[cam]["exposure"]["result"] == "PASS"
        for cam in CAMERAS
    )

    lidar_pass = (
        len(lidar_timebase) > 0
        and 8.0 <= rate_hz(lidar_timebase) <= 12.0
        and non_increasing(lidar_timebase) == 0
        and lidar_empty == 0
        and lidar_point_mismatch == 0
    )
    imu_pass = (
        len(imu_headers) > 0
        and 150.0 <= rate_hz(imu_headers) <= 250.0
        and non_increasing(imu_headers) == 0
    )
    event_gaps = sum(
        max(0, b - a - 1) for a, b in zip(nvpps_events, nvpps_events[1:])
    )
    nvpps_pass = (
        len(nvpps_tsc) >= 2
        and 0.70 <= rate_hz(nvpps_ptp) <= 1.20
        and non_increasing(nvpps_tsc) == 0
        and non_increasing(nvpps_ptp) == 0
        and non_increasing(nvpps_events) == 0
        and event_gaps == 0
        and all(a == b for a, b in zip(nvpps_headers, nvpps_ptp))
    )

    results["lidar"] = {
        "count": len(lidar_timebase),
        "rate_hz": round(rate_hz(lidar_timebase), 6),
        "timebase_non_increasing": non_increasing(lidar_timebase),
        "empty_messages": lidar_empty,
        "point_num_mismatches": lidar_point_mismatch,
        "result": "PASS" if lidar_pass else "CHECK",
    }
    results["imu"] = {
        "count": len(imu_headers),
        "rate_hz": round(rate_hz(imu_headers), 6),
        "header_non_increasing": non_increasing(imu_headers),
        "result": "PASS" if imu_pass else "CHECK",
    }
    results["nvpps"] = {
        "count": len(nvpps_tsc),
        "rate_hz": round(rate_hz(nvpps_ptp), 6),
        "event_gaps": event_gaps,
        "tsc_non_increasing": non_increasing(nvpps_tsc),
        "ptp_non_increasing": non_increasing(nvpps_ptp),
        "header_equals_ptp": all(a == b for a, b in zip(nvpps_headers, nvpps_ptp)),
        "first_tsc_ns": min(nvpps_tsc) if nvpps_tsc else None,
        "last_tsc_ns": max(nvpps_tsc) if nvpps_tsc else None,
        "first_ptp_ns": min(nvpps_ptp) if nvpps_ptp else None,
        "last_ptp_ns": max(nvpps_ptp) if nvpps_ptp else None,
        "result": "PASS" if nvpps_pass else "CHECK",
    }

    common_interval: Dict[str, Any] = {
        "start_ptp_ns": None,
        "end_ptp_ns": None,
        "duration_sec": 0.0,
        "requested_duration_sec": requested_duration_sec,
        "requested_duration_satisfied": False,
    }

    if len(usable_camera_ranges) == len(CAMERAS):
        camera_common_first_tsc = max(item[0] for item in usable_camera_ranges.values())
        camera_common_last_tsc = min(item[1] for item in usable_camera_ranges.values())
        camera_common_first_ptp = interpolate_tsc_to_ptp(
            camera_common_first_tsc, nvpps_tsc, nvpps_ptp
        )
        camera_common_last_ptp = interpolate_tsc_to_ptp(
            camera_common_last_tsc, nvpps_tsc, nvpps_ptp
        )

        if (
            camera_common_first_ptp is not None
            and camera_common_last_ptp is not None
            and lidar_timebase
            and lidar_scan_end
            and imu_headers
        ):
            common_start = max(
                camera_common_first_ptp,
                min(lidar_timebase),
                min(imu_headers),
            )
            common_end = min(
                camera_common_last_ptp,
                max(lidar_scan_end),
                max(imu_headers),
            )
            common_duration = max(
                0.0,
                (common_end - common_start) / 1_000_000_000.0,
            )
            common_ok = (
                requested_duration_sec <= 0.0
                or common_duration >= max(0.0, requested_duration_sec - 0.25)
            )
            common_interval = {
                "start_ptp_ns": common_start,
                "end_ptp_ns": common_end,
                "duration_sec": round(common_duration, 6),
                "requested_duration_sec": requested_duration_sec,
                "requested_duration_satisfied": common_ok,
                "camera_common_start_tsc_ns": camera_common_first_tsc,
                "camera_common_end_tsc_ns": camera_common_last_tsc,
            }

    common_interval_pass = bool(common_interval["requested_duration_satisfied"])
    overall_pass = (
        overall_pass
        and lidar_pass
        and imu_pass
        and nvpps_pass
        and common_interval_pass
    )
    all_intrinsics = all(
        item["status"] == "available"
        for item in calibration.values()
    )
    all_extrinsics = all(
        item["status"] == "available"
        for item in extrinsics.values()
    )

    overall_pass = overall_pass and all_intrinsics
    overall_pass = overall_pass and readout_ready

    duration_sec = (
        (bag_record_max - bag_record_min) / 1_000_000_000.0
        if bag_record_min is not None and bag_record_max is not None
        else 0.0
    )
    bag_bytes = sum(path.stat().st_size for path in bag.glob("*.mcap"))

    validation = {
        "status": "READY" if overall_pass else "NOT_READY",
        "dataset_path": str(dataset),
        "bag_duration_sec": round(duration_sec, 6),
        "bag_size_gib": round(bag_bytes / (1024.0 ** 3), 6),
        "minimum_anchor_coverage_required": args.minimum_anchor_coverage,
        "usable_common_interval": common_interval,
        "streams": results,
        "exposure_ready": all_exposure_pass,
        "exposure_trigger_frame_association": "verified",
        "exposure_optical_application_frame_status": "unverified",
        "intrinsics_available": all_intrinsics,
        "extrinsics_available": all_extrinsics,
        "projection_ready": False,
        "fsync": fsync_config,
        "readout_delay": readout_config,
    }

    (validation_dir / "validation.yaml").write_text(
        yaml.safe_dump(validation, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    (validation_dir / "validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    timing_dir = dataset / "timing"
    timing_dir.mkdir(parents=True, exist_ok=True)
    timing_config = {
        "camera_clock": "Jetson TSC",
        "lidar_clock": "PTP",
        "mapping_source_topic": NVPPS_TOPIC,
        "camera_timestamp_source_field": "raw_v4l2_timestamp_ns",
        "mapping_model": "piecewise interpolation between NvPPS anchors",

        # Rolling-shutter geometry / timing
        "source_image_height": 1860,
        "output_image_height": 1240,
        "row_period_ns": 12_500,

        # 1860 active lines × 12.5 us
        "active_block_ns": 23_250_000,

        # First active-row center -> last active-row center
        # (1860 - 1) × 12.5 us
        "row_center_span_ns": 23_237_500,

        "row_readout_direction": "top_to_bottom",
        "row_readout_direction_status": (
            "configured_not_directly_optically_verified"
        ),

        # Effective raw-SOF -> source row-0 readout anchors.
        # DERIVED/CALIBRATED from measured raw-SOF phase,
        # the C2 timing diagram, and the calibrated ReadOutDelay profile.
        # ReadOutDelay must NOT be added again in the Offline App.
        "raw_sof_to_row0_readout_offset_ns": {
            "cam0": -4_126_856,
            "cam1": -7_106_768,
            "cam2": -7_106_332,
            "cam3": -7_106_572,
        },
        "raw_sof_to_row0_readout_offset_status": (
            "derived_calibrated"
        ),
        "raw_sof_to_row0_readout_offset_basis": (
            "2026-08-14 v5 timestamp finalization"
        ),

        "exposure_reference_status": "per_frame_current_shutter_recorded",
        "exposure_message_type": EXPOSURE_TYPE,
        "exposure_raw_count_unit_ns": EXPOSURE_RAW_COUNT_UNIT_NS,
        "exposure_frame_join_key": (
            "CameraExposureDebug.frame_sequence "
            "<-> CameraTimestampDebug.sequence"
        ),
        "exposure_trigger_frame_association": "verified",
        "exposure_optical_application_frame_status": "unverified",
        "exposure_topics": EXPOSURE_TOPICS,
        "exposure_validation": {
            cam: {
                "result": results[cam]["exposure"]["result"],
                "count": results[cam]["exposure"]["count"],
                "join_ratio": results[cam]["exposure"][
                    "timestamp_debug_join_ratio"
                ],
            }
            for cam in CAMERAS
        },
        "header_to_exposure_center_ns": None,
        "fsync_enabled": fsync_enabled,
        "fsync": fsync_config,
        "readout_delay": readout_config,
        "usable_common_interval": common_interval,
    }
    (timing_dir / "camera_timing.yaml").write_text(
        yaml.safe_dump(timing_config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    camera_manifest = {}
    for cam, info in CAMERAS.items():
        camera_manifest[cam] = {
            "direction": info["direction"],
            "frame_id": info["frame_id"],
            "image_topic": JPEG_TOPICS[cam],
            "message_type": "sensor_msgs/msg/CompressedImage",
            "encoding": "jpeg",
            "jpeg_quality": 95,
            "resolution": [1920, 1240],
            "timestamp_debug_topic": DEBUG_TOPICS[cam],
            "timestamp_source_field": "raw_v4l2_timestamp_ns",
            "intrinsics": calibration[cam]["path"],
            "extrinsics": extrinsics[cam]["path"],
            "exposure": {
                "topic": EXPOSURE_TOPICS[cam],
                "message_type": EXPOSURE_TYPE,
                "source": "GW5300 Current Shutter",
                "raw_count_unit_ns": EXPOSURE_RAW_COUNT_UNIT_NS,
                "frame_join_key": "frame_sequence",
                "trigger_frame_join_result": (
                    results[cam]["exposure"]["result"]
                ),
                "optical_application_frame_status": "unverified",
            },
            "timing": {
                "readout_delay": {
                    "enabled": readout_enabled,
                    "ready": readout_ready,
                    **(
                        readout_cameras.get(cam)
                        if isinstance(
                            readout_cameras.get(cam),
                            dict,
                        )
                        else {}
                    ),
                },
            },
        }

    manifest = {
        "schema_version": 1,
        "dataset": {
            "name": dataset.name,
            "status": "READY" if overall_pass else "NOT_READY",
            "purpose": request.get("purpose", "unspecified"),
            "created_at": request.get("created_at"),
            "requested_duration_sec": request.get("requested_duration_sec"),
            "recorded_duration_sec": round(duration_sec, 6),
            "platform": "Jetson AGX Orin Devkit",
            "ros_distribution": "humble",
            "fsync_enabled": fsync_enabled,
            "readout_delay_enabled": readout_enabled,
            "per_frame_exposure_enabled": True,
            "per_frame_exposure_ready": all_exposure_pass,
        },
        "bag": {
            "path": "bag",
            "storage_id": "mcap",
        },
        "cameras": camera_manifest,
        "lidar": {
            "topic": LIDAR_TOPIC,
            "message_type": "livox_ros_driver2/msg/CustomMsg",
            "timestamp_source": "timebase + point.offset_time",
            "clock": "PTP",
        },
        "imu": {
            "topic": IMU_TOPIC,
            "message_type": "sensor_msgs/msg/Imu",
            "clock": "PTP",
        },
        "time_sync": {
            "nvpps_topic": NVPPS_TOPIC,
            "camera_clock": "TSC",
            "target_clock": "PTP",
            "conversion": "NvPPS TSC-to-PTP anchors",
            "fsync": fsync_config,
            "readout_delay": readout_config,
            "usable_anchor_range": {
                "first_tsc_ns": min(nvpps_tsc) if nvpps_tsc else None,
                "last_tsc_ns": max(nvpps_tsc) if nvpps_tsc else None,
                "first_ptp_ns": min(nvpps_ptp) if nvpps_ptp else None,
                "last_ptp_ns": max(nvpps_ptp) if nvpps_ptp else None,
            },
            "usable_common_interval": common_interval,
        },
        "calibration": {
            "bundle_manifest": (
                "calibration/CALIBRATION_MANIFEST.yaml"
            ),
            "intrinsics": calibration,
            "extrinsics": extrinsics,
        },
        "readiness": {
            "offline_ingest_ready": overall_pass,
            "time_sync_ready": overall_pass,
            "projection_ready": False,
            "missing_for_projection": (
                (
                    []
                    if all_extrinsics
                    else ["LiDAR-camera extrinsics"]
                )
                + [
                    "Auto-exposure optical frame association",
                    "Optical row readout direction verification",
                ]
            ),
        },
        "validation": {
            "summary": "validation/validation.yaml",
        },
    }
    (dataset / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    if readout_enabled:
        readout_summary_parts = []

        for cam in CAMERAS:
            camera_delay = readout_cameras.get(cam)
            if not isinstance(camera_delay, dict):
                readout_summary_parts.append(f"{cam}=missing")
                continue

            requested_value = camera_delay.get(
                "requested_read_out_delay_ms"
            )
            readback_value = camera_delay.get(
                "readback_display_ms"
            )

            readout_summary_parts.append(
                f"{cam}={requested_value}->{readback_value}"
            )

        readout_summary_detail = ", ".join(
            readout_summary_parts
        )
    else:
        readout_summary_detail = "disabled by request"

    summary_lines = [
        "============================================================",
        "OFFLINE DATASET FINALIZATION",
        "============================================================",
        f"Dataset            : {dataset}",
        f"Bag duration       : {duration_sec:.3f} s",
        f"Bag size           : {bag_bytes / (1024.0 ** 3):.3f} GiB",
    ]
    for cam in CAMERAS:
        item = results[cam]
        summary_lines.append(
            f"{cam} {item['direction']:<5}       : {item['result']} "
            f"(jpeg={item['image_count']}, debug={item['debug_count']}, "
            f"pair={item['image_debug_pair_ratio']:.6f}, "
            f"usable={item['anchor_covered_duration_sec']:.3f}s, "
            f"edge_drop={item['edge_frames_before_first_anchor']}+"
            f"{item['edge_frames_after_last_anchor']})"
        )
    for cam in CAMERAS:
        exp = results[cam]["exposure"]
        summary_lines.append(
            f"{cam} Exposure     : {exp['result']} "
            f"(count={exp['count']}, "
            f"rate={exp['rate_hz']:.3f}Hz, "
            f"invalid={exp['invalid_count']}, "
            f"retry={exp['retry_ratio'] * 100.0:.2f}%, "
            f"queue_max={exp['max_queue_depth']}, "
            f"join={exp['timestamp_debug_join_ratio']:.6f})"
        )

    summary_lines.extend([
        f"LiDAR              : {results['lidar']['result']}",
        f"IMU                : {results['imu']['result']}",
        f"NvPPS              : {results['nvpps']['result']}",
        f"Exposure           : {'PASS' if all_exposure_pass else 'CHECK'}",
        f"Common usable      : {common_interval['duration_sec']:.3f} s "
        f"(requested {requested_duration_sec:.3f} s)",
        f"Intrinsics         : {'PASS' if all_intrinsics else 'CHECK'}",
        (
            f"FSYNC              : "
            f"{'ENABLED' if fsync_enabled else 'DISABLED'} "
            f"({fsync_config.get('frequency_hz')} Hz, "
            f"{fsync_config.get('source')})"
        ),
        (
            f"ReadOutDelay       : {readout_result} "
            f"({readout_summary_detail})"
        ),
        f"Extrinsics         : {'PASS' if all_extrinsics else 'CHECK'}",
        f"FINAL RESULT       : {'READY' if overall_pass else 'NOT_READY'}",
        "Projection ready   : NO (timing/exposure semantics pending)",
        "============================================================",
    ])
    summary = "\n".join(summary_lines) + "\n"
    # Keep the detailed report on disk; show a shorter summary in the terminal.
    (validation_dir / "result.txt").write_text(summary, encoding="utf-8")
    print("Dataset validation")
    print(f"  Dataset: {dataset.name}")
    print(f"  Bag: {duration_sec:.3f} s, {bag_bytes / (1024.0 ** 3):.3f} GiB")
    print("  Camera   Frames   Rate (Hz)   Pair     Result")
    for cam in CAMERAS:
        item = results[cam]
        print(
            f"  {cam:<8} {item['image_count']:>6}   "
            f"{item['image_rate_hz']:>8.3f}   "
            f"{item['image_debug_pair_ratio']:.4f}   {item['result']}"
        )
    print(
        f"  LiDAR: {results['lidar']['result']}  "
        f"IMU: {results['imu']['result']}  "
        f"NvPPS: {results['nvpps']['result']}  "
        f"Exposure: {'PASS' if all_exposure_pass else 'CHECK'}"
    )
    print(
        f"  Usable interval: {common_interval['duration_sec']:.3f} s "
        f"(requested {requested_duration_sec:.3f} s)"
    )
    print(f"  Result: {'READY' if overall_pass else 'NOT_READY'}")
    print(f"  Full report: {validation_dir / 'result.txt'}")

    if overall_pass:
        ready.touch()
        return 0

    not_ready.touch()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
