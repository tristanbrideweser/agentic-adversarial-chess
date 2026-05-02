#!/usr/bin/env bash
# =============================================================================
# chess-robots/docker/dev.bash
#
# Convenience wrapper around run.bash.
# Opens an interactive dev shell with the repo mounted.
#
# Equivalent to the lab's run_dev.bash pattern.
#
# Usage:
#   ./dev.bash                         # shell in dev container
#   ./dev.bash "ros2 launch ..."       # run one command then exit
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"

echo -e "\033[2;37mWorkspace: ${REPO_DIR} → /root/chess_ws/src\033[0m"

exec "${SCRIPT_DIR}/run.bash" "${@}"