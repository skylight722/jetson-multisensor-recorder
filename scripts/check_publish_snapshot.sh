#!/usr/bin/env bash
# Static-only gate. Never starts/stops containers or interacts with sensors.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

bash "$ROOT/scripts/check_source_snapshot.sh"

echo 'Checking for unexpected binary data and archived recordings...'
python3 - "$ROOT" <<'PY_CHECK'
from pathlib import Path
import sys
root = Path(sys.argv[1])
ignore_names = {'.git', '__pycache__'}
excluded_extensions = {'.mcap', '.db3', '.bag', '.pem', '.key', '.p12', '.pfx'}
issues = []
for p in root.rglob('*'):
    if any(k in ignore_names for k in p.relative_to(root).parts):
        continue
    if not p.is_file():
        continue
    if p.suffix.lower() in excluded_extensions:
        issues.append(f'Unexpected restricted artifact: {p.relative_to(root)}')
    if p.stat().st_size > 50*1024*1024:
        issues.append(f'Unexpected large file: {p.relative_to(root)}')
for i in issues: print('FAIL:', i)
if issues: sys.exit(1)
print('PASS no disallowed recording/key file extensions and no file over 50 MiB')
print('NOTE: this is not a full license/credential/security audit.')
PY_CHECK
