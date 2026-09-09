# Flying a Crazyflie on Vicon — student guide

You have been given two repositories, a Crazyflie, a radio, and a scene.
This guide gets you from a box of parts to a drone hovering under motion
capture. What you do *after* that — training a policy and flying it — is in
`splat_hitl/WORKFLOW.md`.

Read §2.1 before you touch anything. It is the one idea in this guide that
will save you a drone.

---

## Contents

1. [The drone](#1-the-drone)
2. [Motion capture](#2-motion-capture)
3. [Bring-up](#3-bring-up)
4. [Preflight](#4-preflight)
5. [When it goes wrong](#5-when-it-goes-wrong)

---

# 1. The drone

## 1.1 What you have

A **Crazyflie 2.1+**: 27 g all-up, 92 × 92 × 29 mm motor-to-motor, about
**7 minutes** of flight on a charge and **15 g** of payload before it stops
being a drone and starts being a paperweight. An STM32F405 runs the flight
code; an nRF51822 runs the radio and power management. The IMU is a BMI088
accelerometer/gyro with a BMP388 barometer.

Fifteen grams is not much. A Flow Deck is 1.6 g, a marker is under a gram, and
a battery you brought yourself might be five grams heavier than the stock one.
Weigh things.

## 1.2 Assembly and care

Follow [Bitcraze's assembly video](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/)
rather than guessing. Four things it is easy to get wrong:

- **Twist the motor wires.** Untwisted, they radiate into the radio.
- **Propellers come in two handednesses.** CW is marked `47-17R`, CCW is
  marked `47-17`. Fit them wrong and the drone will not lift, or will lift and
  immediately flip.
- **Convex side up.** Every propeller. Every time.
- **Motor mounts push in with real force.** If it feels loose, it is loose,
  and a motor that rotates in its mount is a slow yaw drift you will spend an
  afternoon blaming on software.

Before every session: spin each propeller with a finger. It must turn freely
and silently. **The single most common fault on a Crazyflie is hair wound
round a motor shaft** — long hair, carpet fibre, a thread off a sleeve. It
presents as one motor working harder, which presents as drift, which presents
as "the controller is badly tuned".

Cracked or chipped propellers get replaced, not flown. An unbalanced prop is
vibration, and vibration is noise in the IMU.

## 1.3 Batteries

These are single-cell lithium polymer. They are safe when treated properly and
genuinely dangerous when not.

- Charge over USB **with the drone switched on**. The rear-left LED blinks
  while charging and goes solid when done — roughly 40 minutes.
- **Do not charge unattended, and do not charge on anything flammable.** A
  ceramic tile or a LiPo bag, not a bed or a pile of paper.
- **A swollen cell is finished.** Not "nearly finished". Do not charge it, do
  not fly it, tell the lab manager and put it in the disposal bin.
- Do not fly a pack flat. `cf_core.py` has a battery guard that lands the
  drone, and repeatedly discharging past it kills packs.
- A pack that gets hot in normal use is a pack to retire.

If you damage a battery — a puncture, a crush from a crash, a cell that gets
hot — that is not something to quietly bin. Tell someone.

---

# 2. Motion capture

## 2.1 Why motion capture fails hard

A Flow Deck fails **soft**. Lose surface texture under the drone and the
velocity estimate degrades, the drone drifts, you see it and land.

Motion capture fails **hard**. The Kalman filter on the Crazyflie is told the
incoming pose is ground truth, with tight covariance. Feed it one bad pose and
the position innovation does not degrade — it explodes, and the drone commits
fully to going somewhere wrong at speed.

The consequence to carry around with you:

> **A wrong pose is far more dangerous than no pose.** A dropout is detected
> and handled. A confidently wrong pose is obeyed.

Every guard in `cf_core.py` exists because of that asymmetry. None is
decoration, and none of them should be raised to make an experiment work.

## 2.2 Teaching the object in Tracker

Your drone needs markers, and Tracker needs to be taught what they mean.
Marker placement is your problem to solve — the short version is that the
constellation must be **asymmetric** and **not flat**, because a symmetric or
planar arrangement gives the solver two equally good answers and it will pick
between them at random, mid-flight. `marker_geom.py` in this repo will tell
you whether yours is a problem before you spend a battery finding out:

```bash
python3 marker_geom.py --vsk /path/to/your_object.vsk --sweep
```

To create the object (Tracker 3.x; check your version's help if the panes have
moved):

1. Put the drone in the middle of the volume, **level**, on a flat surface.
   Tracker builds the object's axes from the pose it sees at this moment, and
   any tilt you teach it is baked in forever and rotates *with the drone*, so
   it never averages out.
2. Pause tracking in the **Objects** tab.
3. `Alt` + left-drag in the 3D view to select your markers. Check the camera
   rays: **at least three cameras must see every marker**, or the object will
   be fragile at the edges of the volume.
4. Type a name in **Create Object** and click **Create**. Use the name your
   code expects.
5. Sanity-check the axes. Walk the drone around; the object frame should move
   with it and stay level when the drone is level.

If tracking misbehaves later, **delete the object and teach it again**. Editing
a bad template is almost always slower than redoing it.

## 2.3 The three failures you will actually meet

| | symptom | check with | fix |
|---|---|---|---|
| **Template misaligned** | constant yaw offset; Vicon says tilted, IMU says level | `imu_tilt.py`, `tilt_origin_check.py` | delete the object, re-teach it level |
| **Rotational ambiguity** | yaw or roll jumps ~90° or ~180° and comes straight back, at random | `marker_geom.py`, `track_monitor.py` | move a marker — no filter fixes this |
| **Occlusion** | tracking degrades in one part of the volume or at one attitude | `track_monitor.py` marker count | more cameras seeing it, or move a marker |

Gravity is an absolute vertical reference. If Vicon and the accelerometer
disagree about which way is down, **Vicon is wrong** — that is what
`imu_tilt.py` is for.

## 2.4 Watching it live

```bash
python3 track_monitor.py --topic /vicon/<object>/<object> --csv run.csv
```

Reports marker visibility and attitude jumps together, and separates a real
solver flip from the yaw angle blowing up near vertical — a distinction that
matters, because only one of them is a tracking problem.

---

# 3. Bring-up

## 3.1 Radio

Plug in the Crazyradio. If the drone does not appear:

```bash
python3 radio_doctor.py
```

It diagnoses the USB claim, which is the usual cause — the dongle can only be
held by one process, so a stray `cfclient` will lock everyone else out.

The default URI is `radio://0/80/2M/E7E7E7E7E7`. In `cfclient`, press **Scan**
rather than typing it: if two teams are on the same channel and address you
will find that out here rather than in the air.

## 3.2 Docker, and the one setting that matters

Everything runs in a container so that your machine and the lab machine agree.

```bash
export USER_UID=$(id -u) USER_GID=$(id -g) \
       PLUGDEV_GID=$(getent group plugdev | cut -d: -f3)
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm cf
```

The image runs the full test suite while it builds, so an image that exists is
an image that passed its own tests.

**Set `ROS_DOMAIN_ID` to your team's number, and never to zero.**

```bash
ROS_DOMAIN_ID=7 docker compose -f docker/docker-compose.yml run --rm cf
```

Every team on domain 0 sees every other team's topics **and setpoints**. That
is not a tidiness issue. It means your drone can be commanded by someone
else's code. Safe values on Linux are 0–101 and 215–232; take the number you
are assigned.

If topics do not appear, the first thing to check is that the domain matches
between your bridge and your node. It is silent when it is wrong.

## 3.3 The bridge and the driver

```bash
# terminal 1 — Vicon into ROS
ros2 launch vicon_receiver client.launch.py

# terminal 2 — the radio and every safety guard
ros2 launch crazyflie_ros crazyflie.launch.py no_fly:=true
```

`no_fly:=true` runs the whole stack with the motors inhibited. Use it the
first time, every time you change something structural, and any time you are
not certain.

The bridge prints a line like this every few seconds:

```
link: latency mean 4.56 ms, max 7.3 ms | dropped 4 of 1200 frames (0.333%)
```

Learn what normal looks like on your rig. A jump in dropped frames is a
network problem, and it will reach the drone as pose holes.

## 3.4 Your first flight

Keyboard teleop, in the net, with one hand on the kill:

```bash
python3 crazyflie_vicon_teleop.py --help
```

Take off, hover for thirty seconds, land. Do not skip this because the
simulator worked. You are checking that the drone, the radio, the tracking and
the safety core all agree about where the drone is.

---

# 4. Preflight

Run this before every session, with the bridge up and the drone sitting still
in the volume:

```bash
python3 preflight.py --topic /vicon/<object>/<object>
```

It checks five things, and every one of them corresponds to something that has
already failed silently on this rig:

| check | why it exists |
|---|---|
| `tcp rto` on the live socket | the low-latency route vanished once and was reported healthy by `systemctl` for two days |
| publish rate at the subscriber | Tracker's configured rate is not evidence of the rate you receive |
| `now − header.stamp` | latency read as 0 for weeks because the bridge stamped pull time, not capture time |
| marker completeness | the telemetry that reports it had never actually executed |
| no attitude branch jumps | a still drone must not reorient |

**Exit code 0 or you do not fly.** If a check fails, the message says what to
do about it. Do not work around it — every one of these was found the
expensive way.

---

# 5. When it goes wrong

| what you see | most likely | not |
|---|---|---|
| Drifts steadily in one direction | prop fouled, prop cracked, motor loose in its mount | controller gains |
| Yaw slowly walks | motor rotating in its mount, or template taught tilted | integrator windup |
| Sudden violent departure | a bad pose was obeyed — check `track_monitor.py` for a flip | "it went unstable" |
| Won't arm, or lands instantly | a guard fired. Read the message; it names itself | a bug |
| Topics missing | `ROS_DOMAIN_ID` mismatch | DDS being mysterious |
| Radio won't connect | another process holds the dongle | a broken radio |
| Works in sim, not in flight | the contract — see `splat_hitl/WORKFLOW.md` §5 | the policy being bad |

The last row is the one that will cost you the most time, so it has its own
section in the workflow guide. A policy that fails because it was trained
through a different camera and a policy that fails because it is bad look
*exactly* the same from outside: both fly into things.

## Asking for help well

Bring these, and most problems are diagnosed in a minute:

- the `preflight.py` output
- the bridge's `link:` line
- a `track_monitor.py` CSV covering the failure
- the run log the flight wrote

---

**Sources for the hardware figures:**
[Crazyflie 2.1+ product page](https://www.bitcraze.io/products/crazyflie-2-1-plus/) ·
[Getting started with the Crazyflie 2.x](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/) ·
[Vicon Tracker documentation](https://help.vicon.com/space/Tracker39/14060303/Creating+an+object)
