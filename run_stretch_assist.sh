#!/usr/bin/env bash
# Launch Stretch Assist with rendering on the NVIDIA dGPU instead of the Intel
# iGPU. The laptop is in PRIME "on-demand" mode, so GL apps render on Intel by
# default; these env vars offload the GLFW viewer + MuJoCo offscreen cameras to
# the RTX 5060, which is much faster (and, unlike the Intel "xe" driver, can
# render multiple offscreen cameras at once, so --show-wrist works).
#
# Usage: ./run_stretch_assist.sh --target glass --no-teleop [other state_machine.py args]
set -euo pipefail
cd "$(dirname "$0")"

# PRIME render offload (GLX, used by MuJoCo's GLFW window + offscreen renderer).
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
# Vulkan offload as well, in case any dependency uses it.
export __VK_LAYER_NV_optimus=NVIDIA_only

exec .venv/bin/python state_machine.py "$@"
