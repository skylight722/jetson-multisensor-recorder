#!/usr/bin/env python3
"""Static repository consistency checks (no ROS, Docker, or hardware access)."""
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
REQUIRED = (
    "README.md",
    "recording_stack/time_sync_stack_v6_fsync.sh",
    "recording_stack/time_sync_stack_v6_core.sh",
    "recording_stack/container_component_v6_fsync.sh",
    "recording_stack/finalize_offline_dataset_v6_fsync.py",
    "ros2_ws/src/ros2_v4l2_camera/package.xml",
    "ros2_ws/src/time_sync_msgs/package.xml",
    "ros2_ws/src/nvpps_ros_publisher/package.xml",
    "ros2_ws/src/c2_exposure_sampler/package.xml",
    "host_setup/fsync_tools/tsc_trigger.c",
    "config/livox/MID360_config.example.json",
)
errors = []
warnings = []

for rel in REQUIRED:
    if not (ROOT / rel).is_file():
        errors.append(f"Missing required file: {rel}")

# Markdown links to local files should resolve inside this repository.
for md in ROOT.rglob("*.md"):
    if ".git" in md.relative_to(ROOT).parts:
        continue
    for raw in re.findall(r"\]\(([^)]+)\)", md.read_text(encoding="utf-8", errors="replace")):
        target = raw.strip().split("#", 1)[0].split(" ", 1)[0]
        if not target or target.startswith(("http:", "https:", "mailto:", "#")):
            continue
        if not (md.parent / target).exists():
            errors.append(f"Broken local link: {md.relative_to(ROOT)} -> {target}")

# Git on Windows should not convert shell scripts to CRLF.
for suffix in (".sh", ".py", ".c", ".cpp", ".h", ".hpp"):
    for path in ROOT.rglob(f"*{suffix}"):
        if ".git" in path.relative_to(ROOT).parts:
            continue
        if b"\r\n" in path.read_bytes():
            errors.append(f"CRLF in Unix source: {path.relative_to(ROOT)}")

# Contacts are still placeholders, not public maintainers' addresses.
for pkg in (ROOT / "ros2_ws/src").glob("*/package.xml"):
    try:
        maintainers = ET.parse(pkg).getroot().findall("maintainer")
    except ET.ParseError as exc:
        errors.append(f"Invalid package.xml: {pkg.relative_to(ROOT)}: {exc}")
        continue
    for maintainer in maintainers:
        email = maintainer.attrib.get("email", "")
        if email.endswith(("@todo.todo", "@example.com", "@example.org")):
            warnings.append(f"Placeholder maintainer email: {pkg.parent.name} ({email})")

if (ROOT / "ros2_ws/src/ros2_v4l2_camera/.gitlab-ci.yml").exists():
    warnings.append("Inherited GitLab CI targets ROS Galactic; this is not a Humble build test")
if not (ROOT / ".gitattributes").exists():
    warnings.append("Missing .gitattributes; check Windows LF/CRLF conversion")
if not (ROOT / "docker/Dockerfile").exists():
    warnings.append("No reproducible Dockerfile yet; existing deployment remains required")

for msg in errors:
    print("ERROR:", msg)
for msg in warnings:
    print("WARN:", msg)
if errors:
    print(f"Repository audit failed: {len(errors)} error(s), {len(warnings)} warning(s)")
    sys.exit(1)
print(f"Repository structure: PASS ({len(warnings)} known warning(s))")
print("This is a static check, not a ROS build or recording test.")
