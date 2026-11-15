#!/bin/bash
# Launcher for LIO-SAM Studio: sources ROS 2 + this workspace, then starts the GUI.
# Desktop icons don't run ~/.bashrc, so the ROS environment must be set up here.
APP_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WORKSPACE_DIR="$(dirname "$APP_DIR")"
LOG_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/lio-sam-studio"
mkdir -p "$LOG_DIR"

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE_DIR/install/setup.bash"
fi

cd "$WORKSPACE_DIR" || exit 1
exec python3 "$APP_DIR/lio_sam_studio.py" "$@" >>"$LOG_DIR/studio.log" 2>&1
