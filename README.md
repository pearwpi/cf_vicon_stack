# cf_vicon_stack

The Crazyflie mocap loop, in two halves that now both speak ROS 2:

* **pose in** - Vicon DataStream to a `PoseStamped` topic (`src/vicon_receiver`)
* **commands out** - either the ROS 2 driver (`src/crazyflie_ros`) or the
  standalone script (`crazyflie_vicon_teleop.py`). Both drive the radio through
  `cflib` and both share one safety core.

```
   Vicon ──▶ vicon_receiver ──▶ /vicon/crazyflie1/crazyflie1 (PoseStamped)
                                          │
                    ┌─────────────────────┴─────────────────────┐
                    ▼                                           ▼
        crazyflie_server (ROS 2)                  crazyflie_vicon_teleop.py
          cmd_* ▶ supervisor ▶ radio               keyboard ▶ supervisor ▶ radio
                    ▲                                           │
             teleop node / RL policy                            │
                    └───────────── cf_core.py ──────────────────┘
                              (one set of guards)
```

Copied out of `vicon_ros_ws` on 2026-08-26.

## Layout

```
cf_vicon_stack/
|-- src/
|   |-- vicon_receiver/            ROS 2 Vicon bridge
|   |-- crazyflie_interfaces/      vendored Crazyswarm2 msg/srv subset
|   `-- crazyflie_ros/             ROS 2 driver + teleop
|       |-- crazyflie_ros/crazyflie_server.py   owns the radio and every guard
|       |-- crazyflie_ros/teleop_node.py        keyboard -> cmd_position
|       |-- crazyflie_ros/cf_core.py            copy of ../../../cf_core.py
|       |-- crazyflie_ros/cf_keyboard.py        copy of ../../../cf_keyboard.py
|       |-- config/crazyflie.yaml
|       `-- launch/crazyflie.launch.py
|
|-- cf_core.py                     THE safety core + Vicon capture, shared
|-- cf_keyboard.py                 keyboard backends, shared
|-- crazyflie_vicon_teleop.py      standalone, no ROS on the command side
|-- frame_check.py                 imported by the teleop for --frame-test
|-- hover_diagnostics.py           imported by the teleop for --diagnose
|
|-- preflight.py                   the go/no-go gate before a session
|-- vicon_probe.py                 link health AND rigid-body health, one capture
|-- track_monitor.py               live yaw-flip / marker-loss watch
|-- tilt_origin_check.py           template vs world tilt, by circle fit
|-- imu_tilt.py                    IMU gravity vs Vicon tilt cross-check
|-- marker_geom.py                 rigid-body geometry from the .vsk
|-- radio_doctor.py                Crazyradio USB claim diagnosis
|-- spin_test.py                   props-off vibration test for yaw flips
|
|-- tests/                         four script suites + the pytest duplicate guard
|   `-- fixtures/                  the 2026-08-12 capture they replay
|-- docs/                          findings and the evidence behind them
|-- docker/                        image, compose file, host setup
|-- fix_symlinks.py                keeps the duplicated files identical
|-- DUPLICATES.json                which files those are
|-- crazyflie2.vsk                 the Vicon rigid-body template
|-- GUIDE.md                       lab setup, wiring, first flight
`-- OPERATIONS.md                  the runbook for a session
```

Two structural points that are load-bearing:

**The top-level Python files must stay in one directory.** `crazyflie_vicon_teleop.py`
does a plain `import frame_check`, `import hover_diagnostics`, `from cf_core import ...`
and `from cf_keyboard import ...`, which resolve only if they sit beside it.

**`cf_core.py` and `cf_keyboard.py` exist twice**: at the root, and inside the
ROS package. The root copy is canonical. They used to be symlinks -- one file
on disk, two import paths -- but symlinks do not survive most transfers, and the
USB copy to the lab PC dropped them and left the suite dying on ImportError.
They are real files in both places now.

Drift is prevented by a test instead of by the filesystem. `DUPLICATES.json`
records the pairs and `test_duplicates.py` fails the moment two copies differ,
so a divergence shows up as a red test rather than as the driver and the
standalone silently enforcing different safety limits. After editing a
canonical copy, push it out and check:

    python3 fix_symlinks.py --sync
    python3 -m pytest tests/test_duplicates.py -q

## Build

    cd cf_vicon_stack
    colcon build
    source install/setup.bash

`vicon_receiver`'s CMakeLists picks the vendored SDK and Boost by
`CMAKE_SYSTEM_PROCESSOR`; both `x86_64` and `aarch64` are present, so the same
tree builds on the laptop and on the Jetson Orin Nano.

`crazyflie_ros` needs `cflib` in the same Python environment as `rclpy`
(system Python 3.10 for Humble, not a conda env):

    python3 -m pip install cflib

## Run: ROS 2

    ros2 launch crazyflie_ros crazyflie.launch.py

Arguments: `teleop:=false`, `vicon:=false` (bridge already up), `no_fly:=true`
(connect and inject pose, never spin a motor), `vicon_hostname:=<ip>`.

Everything is namespaced by the `prefix` parameter, default `cf1`.

| Interface | Type | Notes |
|---|---|---|
| `cf1/cmd_position` | `crazyflie_interfaces/Position` | x y z metres, **yaw degrees** |
| `cf1/cmd_hover` | `crazyflie_interfaces/Hover` | vx vy body m/s, **yaw_rate rad/s, sign-flipped** |
| `cf1/cmd_velocity_world` | `crazyflie_interfaces/VelocityWorld` | m/s, **yaw_rate deg/s** |
| `cf1/cmd_full_state` | `crazyflie_interfaces/FullState` | quaternion (x,y,z,w), **angular deg/s** |
| `cf1/status` | `crazyflie_interfaces/Status` | battery, supervisor bitfield |
| `cf1/pose` | `geometry_msgs/PoseStamped` | the **onboard EKF** estimate, for comparison against Vicon |
| `cf1/takeoff` `cf1/land` `cf1/arm` `cf1/stop` `cf1/go_to` `cf1/notify_setpoints_stop` | services | |
| `cf1/emergency` | `std_srvs/Empty` | cuts motors; the drone falls |

Bring it up the first time with `no_fly:=true` and watch `cf1/pose` against the
Vicon topic. They should agree to a few centimetres. If they do not, stop.

### Units: read this before publishing anything

The message definitions are Crazyswarm2's, verbatim, so that this driver stays
swap-compatible with upstream. Upstream's unit conventions are mutually
inconsistent, and they have been reproduced **exactly** rather than tidied:

| Field | Unit |
|---|---|
| `Position.yaw` | degrees |
| `Hover.yaw_rate` | radians/s, **and sign-flipped** on the way out |
| `VelocityWorld.yaw_rate` | degrees/s |
| `FullState.twist.angular` | degrees/s |
| `GoTo.yaw` | radians, despite the `# deg` comment in the `.srv` |

This is deliberate. Normalising them locally would mean that dropping in the
real `crazyflie_server` later silently changes what your policy commands, which
is a far worse failure than an ugly table. If you want a sane interface, put a
converter node in front. `test_crazyflie_ros.py` section [A] pins every one of
these so a well-meaning cleanup cannot quietly land.

### Two things the ROS driver does differently from upstream Crazyswarm2

**Commands are buffered, not relayed.** Upstream calls `cflib` directly from
each `cmd_*` callback. This driver stores the command and transmits from one
50 Hz timer, so the supervisor gets a veto before anything reaches the radio,
the link is rate-limited by construction, and there is exactly one line in the
file that sends a setpoint. The cost is that commands are downsampled to
`control_hz`.

**There is a command watchdog, because the split creates the need for one.**
In the standalone script the operator and the radio share a process: if the
controller dies, the sender dies with it. Split into two nodes, teleop can
crash, the executor can stall, or DDS can partition, and the driver would keep
re-transmitting the last setpoint forever. So after `command_timeout` (0.30 s)
with no new command while flying, the driver stops relaying and holds position.
It does not land and does not kill: this is a "stop listening", not an
emergency. 0.30 s sits deliberately inside the firmware's own 500 ms
`COMMANDER_WDT_TIMEOUT_STABILIZE`, so we decide what happens rather than the
firmware. (Firmware, for reference: 500 ms levels the attitude and holds,
2000 ms cuts the motors.)

## Run: standalone

Unchanged, and still the right tool when ROS itself is the thing that is
broken. Order below is least dangerous first; nothing above the double line
spins a motor.

| Command | Radio | Motors | What it answers |
|---|---|---|---|
| `python3 vicon_probe.py 30` | no | no | Is the rigid body stable, unambiguous, upright? |
| `python3 tilt_origin_check.py 180` | no | no | Is the tilt in the template or the Vicon world frame? |
| `python3 vicon_probe.py --live` | no | no | Streaming rate, latency, dropouts |
| `python3 imu_tilt.py` | yes | no | Does gravity agree with the Vicon tilt? |
| `python3 crazyflie_vicon_teleop.py --check` | no | no | Do the axes and yaw sign match the frame contract? |
| `python3 crazyflie_vicon_teleop.py --frame-test` | no | no | Constant yaw offset, origin offset |
| `python3 crazyflie_vicon_teleop.py --no-fly` | yes | no | Does the EKF track the injected pose? |
| ===== | === | === | ===== |
| `python3 crazyflie_vicon_teleop.py --motor-test` | yes | **yes** | props OFF |
| `python3 crazyflie_vicon_teleop.py --diagnose 30` | yes | **yes** | flies, writes `hover_log.csv` |
| `python3 hover_diagnostics.py hover_log.csv` | no | no | Why it did not hold position |

The standalone moves its setpoint in the drone's **body** frame. The ROS teleop
node moves it in the **world** frame, because a ROS command topic whose meaning
depends on vehicle attitude is a trap for anything that is not a human holding
a key.

## The shared core

`cf_core.py` holds every constant, the flip filter, the geofence, the pre-arm
gate and the supervisor. It is pure: no ROS, no `cflib`, no I/O, and time is
always passed in rather than read. Two programs can now spin these motors, and
a guard that exists in one and not the other is worse than no guard, because
the two paths then behave differently under exactly the conditions where you
can least afford a surprise.

Supervisor severity, worst first: mocap dead (>400 ms) cuts motors; orientation
unusable (>500 ms) lands; command timeout holds; EKF-vs-Vicon divergence
(>35 cm for >300 ms) lands; geofence lands then cuts; critical battery lands.
The ordering is pinned by tests, not by reading order.

## Tests

    python3 tests/test_frame_check.py
    python3 tests/test_tilt_origin_check.py     # 96, incl. the real-capture replay
    python3 tests/test_flip_guard.py            # 34
    python3 tests/test_crazyflie_ros.py         # 80
    python3 -m pytest tests/test_duplicates.py -q

The first four run without ROS, without `cflib` and without hardware; the ROS
ones stub `rclpy` and the message types. That is deliberate, since it means the
guards stay testable on any machine. They are plain scripts that exit non-zero
on failure, not pytest modules, so run them one at a time -- `tests/fixtures/`
holds the 2026-08-12 capture that the tilt suite replays end to end.

They stay four separate files, and four separate processes, on purpose:
`test_flip_guard.py` monkeypatches the global `time.time` to control the clock,
and since `teleop.time` IS the stdlib module, merging the suites would freeze
time for every other one in the process.

**What the tests do not prove:** that the packages build. Treat the first
`colcon build` on a new machine as a real step, not a formality.


## Known issues carried over untouched

Real, and left alone on purpose: the brief was to collect and wrap working
code, not to change its behaviour.

1. **The reported latency is ROS transport only.** `communicator.cpp` stamps
   each message with `this->get_clock()->now()` when the bridge pulls the frame,
   not with the Vicon capture time. Measured median 0.19 ms, which is DDS, not
   mocap. The camera-to-bridge segment is invisible to every consumer. The SDK
   exposes `GetLatencyTotal()` if you want the real figure.

2. **Occlusion is published as a valid pose.** The bridge never reads the SDK's
   `Occluded` flag. Per the vendored header, an occluded segment returns
   translation `[0,0,0]` and quaternion `[1,0,0,0]`, so the topic emits position
   at the Vicon origin with an orientation that is a 180 degree rotation about
   X, at full rate and with unit norm. The staleness watchdogs never fire,
   because the frame is fresh. One partial mitigation exists by accident:
   `tilt_deg_of(1,0,0,0)` is exactly 180, so the pre-arm upright gate refuses to
   arm on an occluded frame. In flight nothing catches it.

3. **`frame_number` is fetched and discarded** in `get_frame()`, so nothing
   downstream can count genuinely dropped Vicon frames.

4. ~~`vicon_pose_subscriber.py` has no rate auto-calibration.~~ **Fixed by the
   merge.** `capture_stats()` now derives the dropout threshold from the
   MEASURED rate. On the 2026-08-12 capture that finds 141 dropouts where the
   old fixed 100 Hz assumption found far fewer.

5. **`SetStreamMode(ClientPull)`** is the highest-latency of the SDK's three
   modes, and `main()` spins `while (rclcpp::ok()) node->get_frame();` with no
   `spin_some`, so parameter and service callbacks are never serviced.

6. **`crazyflie_interfaces` is vendored, not installed.** It is a subset:
   the six messages and six services the driver uses, matching upstream 1.0.6
   field-for-field. Trajectory upload, add/remove logging and connection
   statistics are absent. `sudo apt install ros-humble-crazyflie-interfaces`
   supersedes it with no code change; delete `src/crazyflie_interfaces/` if you
   do.
