#!/usr/bin/env bash
# SPIRE: copy sessions from the Pi's USB data drive to the PC (run in WSL).
#
#   spire-pull                      list the newest sessions on the Pi
#   spire-pull latest               newest bench session
#   spire-pull latest flight        newest flight session
#   spire-pull session_20261105_114927 [more names ...]
#
# Destination: D:\MASTER THESIS\Code repo\deblur_data (one folder per session).
# Override with SPIRE_DATA=/mnt/... ; Pi address with SPIRE_PI=user@host.
set -euo pipefail

PI="${SPIRE_PI:-revan@192.168.1.240}"
DEST="${SPIRE_DATA:-/mnt/d/MASTER THESIS/Code repo/deblur_data}"
REMOTE_ROOT=/mnt/spire_data

remote_list() {  # all session folders, oldest first
  ssh "$PI" "ls -1d $REMOTE_ROOT/bench/session_* $REMOTE_ROOT/flight/*/ 2>/dev/null | sed 's:/*\$::'"
}

if [ ! -d "$DEST" ]; then
  echo "Destination not found: $DEST" >&2
  exit 1
fi

if [ $# -eq 0 ]; then
  echo "Newest sessions on $PI (bench and flight):"
  remote_list | tail -15 | sed "s:^$REMOTE_ROOT/::"
  echo
  echo "Usage: spire-pull latest | latest flight | <session name> [...]"
  exit 0
fi

all="$(remote_list)"
paths=()
if [ "$1" = "latest" ]; then
  kind="${2:-bench}"
  p="$(printf '%s\n' "$all" | grep "^$REMOTE_ROOT/$kind/" | tail -1 || true)"
  [ -n "$p" ] || { echo "No $kind sessions on the Pi." >&2; exit 1; }
  paths+=("$p")
else
  for name in "$@"; do
    p="$(printf '%s\n' "$all" | grep "/$name\$" | tail -1 || true)"
    [ -n "$p" ] || { echo "Not found on the Pi: $name" >&2; exit 1; }
    paths+=("$p")
  done
fi

for p in "${paths[@]}"; do
  echo "== $p -> $DEST/"
  rsync -a --info=progress2 --exclude '_viewer/' "$PI:$p" "$DEST/"
done
echo "Done. Open session_viewer.html and choose: $DEST"
