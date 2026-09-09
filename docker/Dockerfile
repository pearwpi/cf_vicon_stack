# =============================================================================
# cf_vicon_stack -- base image for the Crazyflie / Vicon course
#
# Students get ROS 2 Humble, cflib, the Vicon bridge and the Crazyflie driver
# already built, plus a non-root user whose UID matches theirs so bind-mounted
# files are not root-owned. They add their own layer: see example-student.Dockerfile.
#
# Build context is the STACK ROOT, not this directory:
#     docker build -f docker/Dockerfile -t cfvicon:base .
# docker/docker-compose.yml sets that up for you.
#
# THE RADIO NEEDS A LINUX HOST. Docker Desktop on macOS/Windows cannot pass USB
# through -- and `--device` there fails SILENTLY: the container starts, nothing
# errors, the dongle is just absent. Everything but flying works anywhere.
#
# udev rules go on the HOST, never in this image: udev is a host daemon reachable
# only over a Unix socket. Run docker/host-setup.sh on the machine with the dongle.
# =============================================================================
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

# ros:humble-ros-base is Ubuntu 22.04 Jammy and ALREADY has build-essential,
# colcon, vcstool and an initialised rosdep. Do not apt-install
# python3-colcon-common-extensions, and do not re-run `rosdep init` -- the second
# init fails with "default sources list file already exists".
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# libusb-1.0-0 is pyusb's fallback backend. cflib bundles libusb via the
# libusb-package wheel, but when no wheel matches the platform that installs
# empty and pyusb falls back to the system library. With neither you get
# `usb.core.NoBackendError`, which reads like a permissions problem and is not.
RUN apt-get update && apt-get install --no-install-recommends -y \
        libusb-1.0-0 python3-pip usbutils sudo \
        less nano iproute2 iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# cflib, and the NumPy trap
# -----------------------------------------------------------------------------
# cflib 0.1.32 declares numpy~=2.2, while Humble's rosidl-generated message
# extensions are compiled against NumPy 1.x. Under NumPy 2 they can fail with
# "_ARRAY_API not found", which would break `import geometry_msgs.msg` -- i.e.
# everything. verify.py proves the combination works or fails the build with an
# actionable message. This is why this Dockerfile can be trusted without having
# been run.
# NumPy 2 was expected to break Humble's rosidl extensions with "_ARRAY_API
# not found". Measured 2026-09-01 on ros:humble-ros-base: it does not. cflib
# 0.1.33 pulls numpy 2.2.6, and a float64[36] covariance field round-trips
# through serialize/deserialize as a correct ndarray. Left UNPINNED on purpose
# -- verify.py now exercises that exact path, so a future incompatible pair
# fails the build instead of shipping.
ARG CFLIB_VERSION="cflib"
ARG PIP_EXTRA=""
COPY docker/verify.py /usr/local/bin/verify.py
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir ${CFLIB_VERSION} ${PIP_EXTRA} \
    && python3 /usr/local/bin/verify.py imports

# -----------------------------------------------------------------------------
# Non-root user
# -----------------------------------------------------------------------------
# Jammy has no pre-existing UID 1000 (Ubuntu 24.04 does, so this block needs
# rewriting if the course moves to Jazzy). Match the host user:
#     --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)
#
# PLUGDEV_GID must match the HOST plugdev group NUMERICALLY. Group names mean
# nothing across the boundary and plugdev is not a Debian-reserved GID, so it
# varies per machine. host-setup.sh prints yours.
ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=1000
ARG PLUGDEV_GID=46
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd  --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && (groupadd --gid ${PLUGDEV_GID} plugdev 2>/dev/null || true) \
    && usermod -aG ${PLUGDEV_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# -----------------------------------------------------------------------------
# The stack, built into /opt/cfvicon
# -----------------------------------------------------------------------------
# NOT /ws. Compose bind-mounts the host tree at /ws for editing, and a bind mount
# SHADOWS whatever the image put there -- including install/. Building into a
# separate prefix keeps the compiled workspace visible at runtime no matter what
# is mounted over the editable copy.
ENV CFV_ROOT=/opt/cfvicon
WORKDIR ${CFV_ROOT}

# cf_core.py and cf_keyboard.py FIRST. The ROS package contains relative symlinks
# (crazyflie_ros/cf_core.py -> ../../../cf_core.py) that point out of src/, so if
# src/ were copied first the symlinks would dangle and colcon would build a
# package with no safety core in it.
COPY cf_core.py cf_keyboard.py ${CFV_ROOT}/
COPY src/ ${CFV_ROOT}/src/

# Fail fast if the symlink does not resolve. A context built from a tarball that
# flattened symlinks would give two silently diverging copies of the safety core,
# which is the one failure mode here that is genuinely dangerous.
RUN for f in cf_core.py cf_keyboard.py; do \
      test "$(sha256sum < ${CFV_ROOT}/$f)" \
         = "$(sha256sum < ${CFV_ROOT}/src/crazyflie_ros/crazyflie_ros/$f)" \
      || { echo "*** $f differs between the stack root and the ROS package."; \
           echo "    Fix on the host with:  python3 fix_symlinks.py --sync"; \
           exit 1; }; \
    done

# The vendored Vicon SDK is ~88 MB and its CMakeLists picks the tree by
# CMAKE_SYSTEM_PROCESSOR, which is what makes this image multi-arch.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct+ \
    && rm -rf ${CFV_ROOT}/log

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source ${CFV_ROOT}/install/setup.bash \
    && python3 /usr/local/bin/verify.py packages \
    && ros2 pkg list | grep -E 'vicon_receiver|crazyflie_ros|crazyflie_interfaces'

# Everything else: standalone tools, tests, the capture fixture they replay.
COPY . ${CFV_ROOT}/

# Run all four suites at build time. They stub rclpy and cflib, so they exercise
# the guards without hardware. A student image is therefore never one that failed
# its own tests.
RUN cd ${CFV_ROOT} \
    && for t in tests/test_frame_check.py tests/test_tilt_origin_check.py \
                tests/test_flip_guard.py tests/test_crazyflie_ros.py; do \
         echo "--- $t"; \
         python3 "$t" > /tmp/out 2>&1 \
           || { tail -30 /tmp/out; echo "*** $t FAILED -- aborting build"; exit 1; }; \
         grep -cE '^  PASS' /tmp/out | xargs -I{} echo "    {} checks passed"; \
       done \
    && find ${CFV_ROOT} -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null; \
    true

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chown -R ${USER_UID}:${USER_GID} ${CFV_ROOT}

# -----------------------------------------------------------------------------
# Runtime
# -----------------------------------------------------------------------------
# Fast DDS shared memory is unreliable across the container boundary and is not
# needed to reach the Vicon PC over the LAN. Forcing UDP trades a little
# same-host throughput for behaviour students can predict. Unset it if you later
# care about intra-host bandwidth.
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    ROS_DOMAIN_ID=0 \
    RCUTILS_COLORIZED_OUTPUT=1 \
    PYTHONUNBUFFERED=1 \
    WS=/ws

# /ws is where compose mounts the editable host tree. Create it so the image is
# still usable with no bind mount at all.
RUN mkdir -p ${WS} && chown ${USER_UID}:${USER_GID} ${WS}

USER ${USERNAME}
WORKDIR ${WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
