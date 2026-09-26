#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build git \
  libvulkan-dev vulkan-tools \
  glslang-tools shaderc \
  libwayland-dev wayland-protocols libxkbcommon-dev \
  libx11-dev libxrandr-dev libxi-dev libxinerama-dev libxcursor-dev \
  pkg-config
echo "now: cmake -S . -B build -G Ninja && cmake --build build"
