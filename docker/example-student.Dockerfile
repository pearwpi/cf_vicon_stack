# =============================================================================
# Example student layer. Copy this into your own repo and edit it.
#
#   docker build -f example-student.Dockerfile -t team7:dev .
#
# You inherit: ROS 2 Humble, cflib, vicon_receiver, crazyflie_ros, cf_core,
# a non-root `student` user, and an entrypoint that has already sourced
# /opt/ros/humble and /ws/install.
#
# You do NOT need to re-source ROS, re-run rosdep init, or reinstall colcon.
# =============================================================================
FROM cfvicon:base

# Root only for installs; the base image ends as `student` and so must you.
USER root

# --- your system packages -----------------------------------------------------
# RUN apt-get update && apt-get install --no-install-recommends -y \
#         ffmpeg \
#     && rm -rf /var/lib/apt/lists/*

# --- your Python packages -----------------------------------------------------
# WATCH NUMPY. The base image pins whatever combination made ROS message
# extensions and cflib import together. Installing a package that drags in a
# different NumPy major can break `import geometry_msgs.msg` for everything.
# After any pip install, re-run the base image's own check:
#     python3 -c "import numpy, rclpy, geometry_msgs.msg, cflib.crtp; print('ok')"
#
# RUN python3 -m pip install --no-cache-dir \
#         "torch==2.*" "stable-baselines3" "gymnasium"

# --- your ROS packages --------------------------------------------------------
# Keep your code in a SEPARATE overlay workspace. Do not build into /ws: that is
# the course workspace, it is bind-mounted at runtime, and your build artefacts
# would collide with it.
ENV OVERLAY_WS=/team_ws
RUN mkdir -p ${OVERLAY_WS}/src && chown -R student:student ${OVERLAY_WS}

USER student
# COPY --chown=student:student my_pkg/ ${OVERLAY_WS}/src/my_pkg/
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && source /ws/install/setup.bash \
#     && cd ${OVERLAY_WS} && colcon build --symlink-install

# The base entrypoint sources ${OVERLAY_WS}/install/setup.bash automatically if
# it exists, so nothing else is needed here.
WORKDIR ${OVERLAY_WS}
