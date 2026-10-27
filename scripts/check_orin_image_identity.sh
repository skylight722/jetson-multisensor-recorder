#!/usr/bin/env bash
# Read-only, intentionally avoids `docker history --no-trunc`, `inspect .Config.Env`
# and full build args because these may reveal credentials.
set -Eeuo pipefail
IMAGE="${1:-lidar-camera-projection:integrated-jpeg-final-20260725}"
CONTAINER="${2:-elated_satoshi}"

echo '=== Container identity ==='
docker inspect "$CONTAINER" \
  --format 'name={{.Name}} image_id={{.Image}} image_tag={{.Config.Image}} status={{.State.Status}}'

echo '=== Image platform and provenance ==='
docker image inspect "$IMAGE" \
  --format 'image_id={{.Id}} created={{.Created}} os={{.Os}} arch={{.Architecture}}'

echo '=== Layer ancestry comparison with previous misys image ==='
python3 - "$IMAGE" <<'PYLAYER'
import json, subprocess, sys

def inspect(name):
    try:
        r = subprocess.run(['docker', 'image', 'inspect', name], check=True, capture_output=True, text=True)
        return json.loads(r.stdout)[0]
    except (subprocess.CalledProcessError, ValueError, IndexError):
        return None

current=inspect(sys.argv[1])
old=inspect('misys:3D_xvn')
if current and old:
    newer = current.get('RootFS', {}).get('Layers', [])
    older = old.get('RootFS', {}).get('Layers', [])
    matching = sum(1 for x,y in zip(newer,older) if x==y)
    common_prefix = 0
    for x,y in zip(newer, older):
        if x != y: break
        common_prefix += 1
    print('old_layers=%d current_layers=%d shared_prefix_layers=%d' % (len(older),len(newer),common_prefix))
    print('old_layers_are_current_prefix=%s' % str(bool(older) and newer[:len(older)] == older).lower())
    print('Note: shared layers do not prove which Dockerfile produced the final image.')
else:
    print('SKIP: one or both image tags unavailable')
PYLAYER

echo '=== Image history IDs and sizes (commands intentionally hidden) ==='
docker history --format '{{.ID}} {{.Size}}' "$IMAGE" | head -n 35

echo '=== Candidate build-context dependency files (existence/size only) ==='
CONTEXT="${HOME}/misys-f1fifth/build_context"
for f in Dockerfile .dockerignore build.sh run.sh packages.apt requirements.pip jetson_pip_excluded.txt; do
  if [[ -f "$CONTEXT/$f" ]]; then
    printf 'PRESENT %-28s %s bytes\n' "$f" "$(stat -c%s "$CONTEXT/$f")"
  else
    printf 'MISSING %s\n' "$f"
  fi
done

echo 'No live container started, no hardware configuration changed.'
