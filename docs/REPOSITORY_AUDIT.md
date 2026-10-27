# Repository audit (2026-10-27)

This review covers the **63-file cleaned source archive** provided during preparation for the public repository, together with the user-provided GitHub Desktop screenshots. The live GitHub revision was not downloaded for this audit, so differences created locally or pushed since that archive may not be reflected here.

## Changes in this update

- Corrected the **base 11-topic** failure log in `recording_stack/time_sync_stack_v6_core.sh`. The sensor gate already checked `BASE_TOPIC_COUNT=11`; only the failure message incorrectly said 15. The later full 15-topic gate remains unchanged.
- Added `.gitattributes` to retain LF line endings for Linux scripts and source files, while preserving `* text=auto`.
- Added `scripts/audit_repository.py` to check required paths, local Markdown links, source line endings and metadata warnings without running hardware.
- Added that checker to `scripts/check_publish_snapshot.sh`.

## Confirmed findings

| Finding | Status | Recommended action |
| --- | --- | --- |
| README and internal Markdown links | PASS in examined archive | Recheck after editing or moving files |
| Expected four ROS 2 package manifests | Present; static checks pass | Run a clean Humble build on Orin later |
| Unwanted bags or credentials by filename extension | Not found by existing static checker | Still review contents and Git history before future public pushes |
| Placeholder maintainer emails in three packages | OPEN | Replace with an approved real contact; do not fabricate an address |
| `ros2_v4l2_camera/.gitlab-ci.yml` uses `osrf/ros:galactic-desktop` | OPEN | Remove as stale inherited CI if not used, or replace with a validated Humble workflow |
| `docker/Dockerfile` for final recording image | Missing intentionally | Preserve honest reproducibility notice; reconstruct/test before adding |
| Camera SDK, pinmux, Device Tree, MAX9296 modifications, calibration | Hardware-specific, incomplete | Document and test on a dedicated Orin setup |
| Upstream / third-party licensing | Requires owner review | Keep Tier IV notices; resolve NVIDIA `nvpps_ioctl.h` obligations and authorship before applying a global license |

### Deliberately left untouched

- ReadOutDelay commands and their **integer-only displayed readback checks** (`11`, `0`, `25`, `16`). The requests use fractional values but the original SDK's readback behavior is documented as integer-only; making this comparison equal the requested fractional value would be an unverified behavior change.
- ROS topic names and message types, JPEG capture and encoding, PTP/FSYNC/NvPPS timing code, recording pause/resume logic, data validation calculations.
- `/dev/i2c-*`, `eno1`, `/dev/nvpps0`, `/dev/video*` and board-specific paths. These should be adapted and validated on target hardware, not guessed in an offline cleanup.

## Local verification

From a Linux shell with Python 3:

```bash
python3 scripts/audit_repository.py
bash scripts/check_publish_snapshot.sh
```

On Windows, `python scripts/audit_repository.py` also works if Python 3 is installed. The Bash check requires Bash/Linux, WSL, or a compatible shell. These do **not** prove the sensor stack works on Orin.
