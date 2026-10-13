#!/usr/bin/env bash
# Save a timestamped copy of the sources after a successful build.
# Skips if nothing changed since the latest snapshot.
set -euo pipefail
root="$1"
dest_dir="$root/snapshots"
mkdir -p "$dest_dir"

latest=$(ls -1d "$dest_dir"/*/ 2>/dev/null | sort | tail -n1 || true)
if [ -n "$latest" ] && diff -rq "$root/src" "$latest/src" >/dev/null 2>&1 \
   && cmp -s "$root/CMakeLists.txt" "$latest/CMakeLists.txt"; then
  echo "snapshot: no changes since $(basename "$latest")"
  exit 0
fi

name="build_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$dest_dir/$name"
cp -r "$root/src" "$root/CMakeLists.txt" "$dest_dir/$name/"
echo "snapshot: saved $dest_dir/$name"
