#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$ROOT/shaders}"
mkdir -p "$OUT"
if command -v glslc >/dev/null; then
  glslc -fshader-stage=vert "$ROOT/shaders/mesh.vert" -o "$OUT/mesh.vert.spv"
  glslc -fshader-stage=frag "$ROOT/shaders/mesh.frag" -o "$OUT/mesh.frag.spv"
elif command -v glslangValidator >/dev/null; then
  glslangValidator -V "$ROOT/shaders/mesh.vert" -o "$OUT/mesh.vert.spv"
  glslangValidator -V "$ROOT/shaders/mesh.frag" -o "$OUT/mesh.frag.spv"
else
  echo "install shaderc (glslc) or glslang-tools" >&2
  exit 1
fi
echo "wrote $OUT/mesh.vert.spv $OUT/mesh.frag.spv"
