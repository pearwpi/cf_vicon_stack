# Running cf_vicon_stack in Docker

A base image with ROS 2 Humble, cflib and this whole stack already built. You
build it once, then build your own image `FROM cfvicon:base`.

## Read this first: the radio needs a Linux host

Docker Desktop on **macOS and Windows cannot pass a USB device to a container**
— it needs hypervisor support that does not exist. Worse, `--device` there
*fails silently*: the container starts, nothing errors, and the dongle is simply
absent. Everything except flying works on any host. Flying does not.

If your laptop is not Linux, use it for sim and training, and fly from a Linux
machine.

## One-time host setup

On the Linux machine with the Crazyradio plugged in:

    ./docker/host-setup.sh

It installs `/etc/udev/rules.d/99-bitcraze.rules`, puts you in `plugdev`, shows
what `lsusb` actually sees, and prints the three build args for your machine.
**Log out and back in afterwards** or the group change will not apply.

udev rules must live on the host. udev is a host daemon reachable only over a
Unix socket, so a container can neither install nor trigger them; the ownership
the container sees on `/dev/bus/usb/...` is exactly what host udev stamped there.

## Build and run

    export USER_UID=$(id -u) USER_GID=$(id -g) \
           PLUGDEV_GID=$(getent group plugdev | cut -d: -f3)

    docker compose -f docker/docker-compose.yml build
    docker compose -f docker/docker-compose.yml run --rm cf

The `USER_UID`/`USER_GID` args matter: they make the container user match you,
so files you create in the bind-mounted workspace are yours and not root's.
`PLUGDEV_GID` must match your host's `plugdev` group *numerically* — group names
mean nothing across the boundary, the kernel checks the number, and `plugdev` is
not a Debian-reserved GID so it varies per machine.

Inside:

    ros2 launch crazyflie_ros crazyflie.launch.py no_fly:=true
    python3 vicon_probe.py --live

## Multi-arch (Jetson Orin Nano)

`ros:humble-ros-base` publishes amd64 and arm64, and the vendored Vicon SDK
carries both trees, so one Dockerfile covers laptops and the Jetson:

    docker buildx create --use --name cfvicon 2>/dev/null || docker buildx use cfvicon
    docker buildx build --platform linux/amd64,linux/arm64 \
        -f docker/Dockerfile -t cfvicon:base --load .

Note `osrf/ros:humble-desktop*` (the RViz images) is **amd64 only**, so do not
base an arm64 build on it.

## Give every team a different ROS_DOMAIN_ID

    ROS_DOMAIN_ID=7 docker compose -f docker/docker-compose.yml run --rm cf

Every container on domain 0 on the same LAN sees every other team's topics —
including their setpoints. Use 1–101 (and 215–232); other values collide with
the ephemeral port range on Linux.

## Why the compose file looks like that

| Setting | Why |
|---|---|
| `network_mode: host` | Bridge networking puts the container behind NAT. DDS advertises locators the Vicon PC cannot route back to, and multicast discovery does not traverse the bridge. |
| `- /dev/bus/usb:/dev/bus/usb` as a **volume**, not `devices:` | `--device` grants only the nodes present at container start. Unplug and replug the radio and it gets a new bus/address, hence a new device node the container was never granted. A bind mount is a live view of the host's devtmpfs, so new nodes appear. |
| `device_cgroup_rules: c 189:* rmw` | 189 is the USB char major. The device cgroup would still deny `open()` on a node it did not grant, so this pre-authorises the major. Together with the bind mount it replaces `privileged: true`. |
| `ipc: host`, `shm_size: 512m` | Only needed if you re-enable shared-memory DDS. Docker's default `/dev/shm` is 64 MB. |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | Fast DDS shared memory is unreliable across the container boundary and produces `Failed to create segment ... Permission denied` noise, especially with a root container and a non-root host process. It is not needed to reach the Vicon PC. Predictability beats throughput in a teaching image. |
| `stdin_open` + `tty` | The keyboard teleop puts the terminal into raw mode. |

## Building your own layer

See `example-student.Dockerfile`. Two rules:

1. **Build into an overlay workspace**, not `/ws`. `/ws` is the course workspace
   and is bind-mounted at runtime; your artefacts would collide with it. Set
   `OVERLAY_WS` and the entrypoint sources it for you.
2. **Re-check NumPy after any pip install.** The base pins a combination where
   ROS message extensions and cflib import together. A package that drags in a
   different NumPy major can break `import geometry_msgs.msg` for everything:

       python3 -c "import numpy, rclpy, geometry_msgs.msg, cflib.crtp; print('ok')"

## The build tests itself

The image runs every check from the four suites during `docker build` and fails
if any of them do, so a student image is never one that failed its own tests. It
also verifies that the two copies of `cf_core.py` — the stack root and the ROS
package — are byte-identical. They were symlinks once, and the guard checked
that the link resolved; symlinks do not survive a transfer, so they are real
files now and the guard checks content. The failure mode is unchanged: two
silently diverging copies of the safety core is the one thing here that would be
genuinely dangerous.

## Troubleshooting

**`usb.core.NoBackendError: No backend available`** — pyusb cannot find libusb.
The image installs `libusb-1.0-0` as the fallback. If you see this, you are
probably on a host where `/dev/bus/usb` was not mounted; check the entrypoint's
startup line.

**`Errno 16 Resource busy` on `set_configuration`** — something else holds the
dongle, usually ModemManager on the host. `python3 radio_doctor.py --fix`.

**`ros2 topic list` is empty but the bridge is running on another machine** —
check `ROS_DOMAIN_ID` matches on both ends, that `ROS_LOCALHOST_ONLY` is unset
or 0, and that the interface is multicast-enabled. `ros2 multicast receive` on
one machine and `ros2 multicast send` on the other is the fastest test.

**TF extrapolation errors that look like mocap dropouts** — clock skew between
the container host and the Vicon PC. Run chrony/NTP on both.

**Build aborts with "ROS message extensions and the installed NumPy disagree"** —
that is the guard working. Rebuild with
`--build-arg PIP_EXTRA='numpy<2'` or an older cflib via
`--build-arg CFLIB_VERSION='cflib==0.1.28'`.
