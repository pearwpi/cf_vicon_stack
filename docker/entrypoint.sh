#!/usr/bin/env bash
# Chain the base image's ROS sourcing, then the built stack, then any overlay.
#
# ros:humble's own /ros_entrypoint.sh does `source /opt/ros/$ROS_DISTRO/setup.bash`.
# Overriding ENTRYPOINT without re-sourcing is the most common way to end up with
# "ros2: command not found" in a derived image.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# The COMPILED workspace, at a prefix no bind mount shadows.
if [ -f "${CFV_ROOT:-/opt/cfvicon}/install/setup.bash" ]; then
    source "${CFV_ROOT:-/opt/cfvicon}/install/setup.bash"
fi

# Students layering their own packages point OVERLAY_WS at their workspace.
if [ -n "${OVERLAY_WS}" ] && [ -f "${OVERLAY_WS}/install/setup.bash" ]; then
    source "${OVERLAY_WS}/install/setup.bash"
fi

# So `python3 vicon_probe.py` works from /ws even before anything is mounted.
export PYTHONPATH="${CFV_ROOT:-/opt/cfvicon}:${PYTHONPATH}"

if [ -z "${CFVICON_QUIET}" ]; then
    echo "cf_vicon_stack | ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
         "| transports=${FASTDDS_BUILTIN_TRANSPORTS:-default}"
    echo "  built stack: ${CFV_ROOT:-/opt/cfvicon}   editable copy: ${WS:-/ws}"
    [ -e /dev/bus/usb ] || echo "  note: /dev/bus/usb not mounted -- no radio in this container."
fi
exec "$@"
