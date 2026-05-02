#!/usr/bin/env bash
# =============================================================================
# chess-robots/docker/build.bash
#
# Builds the chess-robots dev Docker image.
#
# Usage:
#   ./build.bash              # build dev image
#   ./build.bash --no-cache   # force full rebuild
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"

IMAGE="chess-robots/dev:latest"

# Platform detection — avoids slow QEMU emulation on Apple Silicon
case "$(uname -m)" in
    arm64|aarch64) PLATFORM="--platform linux/arm64" ;;
    x86_64)        PLATFORM="--platform linux/amd64" ;;
    *)             PLATFORM="" ;;
esac

BUILD_CMD=(
    docker build
    ${PLATFORM}
    --tag "${IMAGE}"
    --file "${SCRIPT_DIR}/dev/Dockerfile"
    "${SCRIPT_DIR}/dev"   # build context = docker/dev/
    "${@}"                # forward any extra args (e.g. --no-cache)
)

echo -e "\033[1;30m${BUILD_CMD[*]}\033[0m"
exec "${BUILD_CMD[@]}"