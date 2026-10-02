# cf_vicon_stack

Flies a Crazyflie under Vicon motion capture, with ROS 2:

* **pose in**: Vicon DataStream → a `PoseStamped` topic (`src/vicon_receiver`)
* **commands out**: the ROS 2 driver (`src/crazyflie_ros`) or the standalone
  script (`crazyflie_vicon_teleop.py`). Both drive the radio through `cflib`
  and share one safety core, `cf_core.py`.

```
   Vicon ──▶ vicon_receiver ──▶ /vicon/<object>/<object> (PoseStamped)
                                          │
                    ┌─────────────────────┴─────────────────────┐
                    ▼                                           ▼
        crazyflie_server (ROS 2)                  crazyflie_vicon_teleop.py
          cmd_* ▶ supervisor ▶ radio               keyboard ▶ supervisor ▶ radio
                    ▲                                           │
             teleop node / policy                               │
                    └───────────── cf_core.py ──────────────────┘
                              (one set of guards)
```

**Course documentation:** <https://pear-wiki.wpi.edu/rbe595/>. It covers
setting up the machine and the container, the drone, the first flight, the
diagnostic tools and the safety limits.

## Layout

```
cf_vicon_stack/
|-- src/
|   |-- vicon_receiver/            ROS 2 Vicon bridge
|   |-- crazyflie_interfaces/      vendored Crazyswarm2 msg/srv subset
|   `-- crazyflie_ros/             ROS 2 driver and keyboard node
|       |-- crazyflie_ros/crazyflie_server.py   owns the radio and every guard
|       |-- crazyflie_ros/teleop_node.py        keyboard -> cmd_position
|       |-- crazyflie_ros/cf_core.py            copy of ../../../cf_core.py
|       |-- crazyflie_ros/cf_keyboard.py        copy of ../../../cf_keyboard.py
|       |-- config/crazyflie.yaml
|       `-- launch/crazyflie.launch.py
|
|-- cf_core.py                     the safety core, shared by both programs
|-- cf_keyboard.py                 keyboard input, shared
|-- crazyflie_vicon_teleop.py      standalone flight script, no ROS on the command side
|-- frame_check.py                 used by the teleop's --frame-test
|-- hover_diagnostics.py           used by the teleop's --diagnose
|
|-- preflight.py                   the go/no-go check before a session
|-- vicon_probe.py                 Vicon link and rigid-body health
|-- track_monitor.py               live heading-jump and marker-loss watch
|-- tilt_origin_check.py           is the tilt in the Vicon object or the floor?
|-- imu_tilt.py                    accelerometer vs Vicon tilt
|-- marker_geom.py                 checks a marker layout from its .vsk
|-- radio_doctor.py                finds what is holding the Crazyradio
|-- spin_test.py                   props-off motor test for heading jumps
|
|-- tests/                         four test scripts and the duplicate-file check
|-- docker/                        image, compose file, host setup (see DOCKER.md)
|-- fix_symlinks.py                keeps the duplicated files identical ...
|-- DUPLICATES.json                ... and lists them
`-- crazyflie2.vsk                 a Vicon object template
```

The top-level Python files must stay in one folder: the teleop imports
`frame_check`, `hover_diagnostics`, `cf_core` and `cf_keyboard` from beside it.

`cf_core.py` and `cf_keyboard.py` exist twice: at the root, which is the copy
to edit, and inside the ROS package. `tests/test_duplicates.py` fails if the
copies differ. After editing the root copy:

    python3 fix_symlinks.py --sync
    python3 -m pytest tests/test_duplicates.py -q

## Build and run

Use the Docker image: see `docker/DOCKER.md`. To build natively instead, on
Ubuntu 22.04 with ROS 2 Humble:

    colcon build
    source install/setup.bash
    python3 -m pip install cflib     # into the same Python as rclpy, not a conda env

    ros2 launch crazyflie_ros crazyflie.launch.py config:=<your copy of crazyflie.yaml>

Launch arguments: `config:=` (your copy of
`src/crazyflie_ros/config/crazyflie.yaml`), `teleop:=false`, `vicon:=false`
(the bridge is already running), `no_fly:=true` (connect and send poses, never
spin a motor), `vicon_hostname:=<ip>`.

The standalone script's test modes (`--check`, `--frame-test`, `--no-fly`,
`--motor-test`, `--diagnose`) are on the course site's First flight page. Its
keyboard moves the setpoint in the drone's body frame; the ROS keyboard node
moves it in the world frame.

## ROS 2 interface

Everything is namespaced by the `prefix` parameter, default `cf1`.

| interface | type | notes |
|---|---|---|
| `cf1/cmd_position` | `crazyflie_interfaces/Position` | x y z metres, **yaw degrees** |
| `cf1/cmd_hover` | `crazyflie_interfaces/Hover` | vx vy body m/s, **yaw_rate rad/s, sign-flipped** |
| `cf1/cmd_velocity_world` | `crazyflie_interfaces/VelocityWorld` | m/s, **yaw_rate deg/s** |
| `cf1/cmd_full_state` | `crazyflie_interfaces/FullState` | quaternion (x,y,z,w), **angular deg/s** |
| `cf1/status` | `crazyflie_interfaces/Status` | battery, supervisor bitfield |
| `cf1/pose` | `geometry_msgs/PoseStamped` | the **onboard estimate**, to compare with Vicon |
| `cf1/takeoff` `cf1/land` `cf1/arm` `cf1/stop` `cf1/go_to` `cf1/notify_setpoints_stop` | services | |
| `cf1/emergency` | `std_srvs/Empty` | cuts the motors; the drone falls |

Bring it up the first time with `no_fly:=true` and compare `cf1/pose` with the
Vicon topic. They should agree to a few centimetres.

**Units.** The message definitions are Crazyswarm2's, unchanged, so the driver
stays interchangeable with upstream, and so are its mixed units:

| field | unit |
|---|---|
| `Position.yaw` | degrees |
| `Hover.yaw_rate` | radians/s, sign-flipped on the way out |
| `VelocityWorld.yaw_rate` | degrees/s |
| `FullState.twist.angular` | degrees/s |
| `GoTo.yaw` | radians, despite the `# deg` comment in the `.srv` |

`tests/test_crazyflie_ros.py` checks each of these.

**Two differences from Crazyswarm2's driver.** Commands are buffered: the driver
stores each command and sends from one 50 Hz timer, so the safety checks see
every command before the radio does. And if no command arrives for 0.30 s while
flying, the driver holds the drone where it is; it does not land. (The
firmware's own watchdog levels the drone after 0.5 s and stops the motors after
2 s.)

## The safety core

`cf_core.py` holds every limit and check: the flip filter, the flight box, the
pre-arm check and the supervisor. It does no I/O and imports neither ROS nor
`cflib`, so it is tested on any machine, and both flight programs use it.

The supervisor's checks, in the order it applies them:

| condition | action |
|---|---|
| no Vicon pose for 0.40 s | cut the motors |
| Vicon orientation unusable for 0.50 s | land |
| no command for 0.30 s | hold position |
| onboard estimate over 0.35 m from Vicon for 0.30 s | land |
| more than 0.80 m outside the flight box | cut the motors |
| more than 0.30 m outside the flight box | land |
| battery below 3.20 V | land |

## Tests

    python3 tests/test_frame_check.py
    python3 tests/test_tilt_origin_check.py
    python3 tests/test_flip_guard.py
    python3 tests/test_crazyflie_ros.py
    python3 -m pytest tests/test_duplicates.py -q

They need no ROS, `cflib` or hardware: the ROS tests stub `rclpy` and the
messages. Run them one at a time, as separate processes: `test_flip_guard.py`
replaces the clock for its whole process. The Docker build runs all four and
stops if any fails.

## Known limitations

1. The bridge publishes an occluded object as the Vicon SDK's placeholder pose
   (the origin, rotated 180° about x). The driver and the standalone script drop
   those frames, so the 0.40 s Vicon watchdog acts on them.
2. The bridge polls the SDK (`ClientPull`) and never services ROS callbacks, so
   its parameters cannot be changed while it runs.
3. `crazyflie_interfaces` is a vendored subset of Crazyswarm2's 1.0.6
   interfaces: six messages and six services. Installing
   `ros-humble-crazyflie-interfaces` replaces it with no code change; then
   delete `src/crazyflie_interfaces/`.
