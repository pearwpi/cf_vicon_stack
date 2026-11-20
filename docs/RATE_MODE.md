# Rate mode: body rates + collective thrust (`cmd_rate`)

Status: implemented, **never flown**. Firmware rate mode is OFF and the
`flightmode.stabModeRoll/Pitch` parameters are deliberately left unchanged
until the owner approves changing them. Nothing in the default configuration
touches them.

## Interface

| topic | type | notes |
|---|---|---|
| `cf1/cmd_rate` (in) | `crazyflie_interfaces/RateThrust` | body rates rad/s + collective thrust N |
| `cf1/imu` (out) | `sensor_msgs/Imu` | gyro rad/s, accel m/s^2, body FLU, gravity included, no orientation (covariance[0] = -1) |
| `cf1/log/thrust` (out) | `crazyflie_interfaces/LogDataGeneric` | `[stabilizer.thrust, pm.vbat]` at 10 Hz, used by `scripts/calibrate_thrust.py` |

`RateThrust` is local to this stack (not Crazyswarm2's) and is plain SI:
nothing in degrees, nothing sign-flipped by the caller.

## Units and frames

- Body frame is FLU, right-handed: x forward, y left, z up.
- `roll_rate`, `pitch_rate`, `yaw_rate`: rad/s about body +x, +y, +z.
- `thrust`: collective thrust, newtons, summed over the four rotors.
- `thrust`: must be > 0. Zero and negative thrust are REJECTED (they used to
  map silently to the 10001 floor word, a spinning-motor command).
- `thrust_pwm`: 0 normally. If > 0 it would override `thrust` with a raw legacy
  thrust word (clamped to `thrust_pwm_min..max`), bypassing the newton map; it
  is REFUSED unless the driver is started with `rate_allow_raw_pwm:=true`
  (default false, bench use only).
- Non-finite fields are rejected (never clamped). Finite out-of-range values
  are saturated to the caps below.
- `imu` is `<prefix>/body` FLU; accel is specific force (+9.81 on z at rest).

## Sign conventions

Derived from cflib and firmware source (see `rate_cmd.py` header):

| field | driver sends to cflib `send_setpoint` | status |
|---|---|---|
| roll_rate | `+degrees(roll_rate)` | TO VERIFY with props off |
| pitch_rate | `+degrees(pitch_rate)` (cflib negates it on the wire, firmware negates it back) | TO VERIFY with props off |
| yaw_rate | `-degrees(yaw_rate)` (firmware inverts legacy yaw rate; same flip as `Hover.yaw_rate`) | TO VERIFY with props off |
| thrust N -> word | per-motor force polynomial inverted, then clamped to 10001..55000 | TO VERIFY (hover word via calibration) |
| IMU axes/signs | gyro/acc straight from `gyro.*`, `acc.*` log vars | TO VERIFY with props off |

## Parameters

Driver parameters (`config/crazyflie.yaml`, all safe by default):

| param | default | meaning |
|---|---|---|
| `enable_rate_cmd` | false | accept `cmd_rate` at all |
| `rate_mode_setup` | false | let the driver WRITE the firmware flight mode (below) |
| `rate_max_rp_dps` / `rate_max_yaw_dps` | 120 / 360 | rate caps (training caps) |
| `rate_tilt_limit_deg` | 60 | tilt guard |
| `rate_att_max_age_s` | 0.10 | max age of the attitude sample rate output may rely on |
| `rate_allow_raw_pwm` | false | allow the raw `thrust_pwm` override (bench only) |
| `takeoff_ground_z_max` | 0.25 | takeoff only from at/below this measured z |
| `thrust_poly`, `thrust_scale` | linear firmware map, 1.0 | newton -> thrust word |
| `thrust_pwm_min/max` | 10001 / 55000 | clamp of the thrust word |
| `enable_imu_log`, `imu_log_hz`, `imu_log_attitude` | true, 100, false | IMU stream |

Startup validation: `cmd_rate` stays disabled (error logged) unless
`thrust_poly` is increasing, `thrust_scale` is finite and > 0, the rate caps are
in (0, 2000] deg/s, the tilt limit is in (0, 180) deg, and
`0 < thrust_pwm_min < thrust_pwm_max <= 65535`.

Firmware parameters that rate mode requires (**NOT changed by default; awaiting
owner approval**):

- `flightmode.stabModeRoll = 0` (RATE; firmware default 1 = angle)
- `flightmode.stabModePitch = 0` (RATE; default 1)
- `flightmode.stabModeYaw` is rate (0) by firmware default; it is only
  verified, never written.

They are written only when `enable_rate_cmd:=true` **and**
`rate_mode_setup:=true`, at takeoff, on the ground. The driver also sends one
zero-thrust RPYT packet at that point, because the firmware RPYT decoder zeroes
thrust until it has seen one (thrust lock).

## Takeoff eligibility, emergency latch

- `takeoff` is accepted only from state IDLE, with measured z at or below
  `takeoff_ground_z_max` (0.25 m), a finite target height inside the flight
  volume, and flight-mode state not uncertain (below). All of this is checked
  BEFORE anything is armed, unlocked or written, so a second `takeoff` while
  airborne does nothing (it used to send a zero-thrust packet mid-air).
  `height == 0` means "use `hover_z`"; NaN and negative are refused.
- An emergency (service or supervisor kill) latches state STOPPED. `stop` does
  not clear it, `arm:=true` is refused, and `takeoff` is refused until the
  `<prefix>/reset_emergency` service (std_srvs/Empty) is called.
- `arm:=true` is refused unless state is IDLE and flight-mode state is certain.

## Flight-mode bookkeeping and revert

The driver keeps a restore debt per param (`stabModeRoll`, `stabModePitch`, and
`stabModeYaw` if it was found non-default). A debt is created BEFORE the write,
so a write that throws or is lost still leaves it. It is cleared only when the
firmware reports the default value AFTER our last write (param update callback),
and unacknowledged writes are re-sent every 0.5 s by the control tick (no
waiting in the tick; it only reads the cache and queues writes).

- Restored on: landing complete, `stop`, emergency (after motors are cut), and
  shutdown. Shutdown waits up to 1.5 s for the acknowledgments, retrying, and
  logs a loud error naming the params if they never arrive (power-cycle then).
- At connect ANY non-default combination (roll/pitch not 1, yaw not 0, mixed
  states, all-rate with an unexpected yaw) is restored, whatever
  `rate_mode_setup` says, and tracked until acknowledged.
- While a debt is unacknowledged, or (with `enable_rate_cmd`) a param is
  unreadable or non-default, or the link was lost, `takeoff` and `arm:=true`
  are refused.
- Radio link lost: the cflib `connection_lost`/`disconnected` callbacks clear
  the command buffer, drop the unlock, latch STOPPED and mark the mode unknown
  (takeoff stays refused until a reconnect). The firmware watchdog stops the
  motors; the params stay in RAM until a restore is acknowledged or a reboot.

## Safety behaviour

- `cmd_rate` is accepted only if: `enable_rate_cmd`, `rate_mode_setup`, state
  is FLYING, and the rate handoff is verified (checked every tick):
  - `stabModeRoll/Pitch/Yaw = 0`, each REPORTED by the firmware within 1.5 s
    (write ack or readback; the driver re-reads them every 0.5 s while flying);
  - the decoder-affecting params `flightmode.althold`, `flightmode.poshold`,
    `flightmode.posSet` read back as 0, same freshness. They change how the
    legacy packet is interpreted (thrust as climb rate, roll/pitch as
    velocities). A param missing from the firmware TOC counts as NOT verified:
    refused;
  - the thrust unlock (zero-thrust packet) was sent on the ground at this
    takeoff and the setpoint stream has not been interrupted for more than 1 s
    (the firmware re-locks thrust when its commander times out). stop,
    emergency and link loss clear it.
  Otherwise it is refused with a throttled warning, not stored, and does not
  refresh the command watchdog. If any of this is lost while flying rates, no
  rate packet is sent and the driver holds position.
- Stale command (no message for `command_timeout`, 0.30 s): HOLD position.
  The supervisor evaluates geofence, EKF mismatch and battery BEFORE this hold,
  so a policy outage never masks them (fence -> land/kill, battery -> land).
- Attitude: every rate packet needs a fresh (<= `rate_att_max_age_s`, 0.10 s)
  VALID attitude: the last Vicon quaternion the flip filter accepted and/or the
  onboard roll/pitch (`imu_log_attitude`). A stale mocap pose (> 150 ms), no
  fresh valid attitude, or tilt over `rate_tilt_limit_deg` on any fresh
  source stops rate output immediately and LANDS (the most conservative defined
  action: a position hold would trust the pose/attitude that just went
  missing). Mocap dead > 400 ms still cuts the motors.
- Rate and thrust caps as above; thrust word clamped to 10001..55000.
- All existing supervisor checks (Vicon loss, flight box, battery) still apply.
- A new rate command cancels the post-takeoff HOLD point; landing is not
  overridable by it.

RESIDUAL RISK: the driver cannot stop another cflib client (cfclient, a second
script) from writing `flightmode.*` after our last readback. The 0.5 s refresh
and 1.5 s freshness window bound how long a changed decoder or flight-mode
param can go unnoticed, but not eliminate it. Run nothing else against the
drone during rate flight. Param-update callbacks rely on cflib's
`add_update_callback`; if a cflib build does not deliver them, freshness can
never be proven and rate flight is refused (fail safe).

## Thrust calibration

`scripts/calibrate_thrust.py` hovers in the ordinary position mode (it refuses
to run if `enable_rate_cmd` or `rate_mode_setup` is true), records
`stabilizer.thrust` and battery voltage, estimates the hover word with a
trimmed mean, and writes a YAML (`thrust_poly`, `thrust_scale`) that you pass to
the driver as a second `--params-file`. Weigh the drone as flown.

    python3 scripts/calibrate_thrust.py --dry-run                      # on the ground, no takeoff
    python3 scripts/calibrate_thrust.py --dry-run --no-ros --hover-pwm 38000 --mass 0.034 --out cal.yaml
    python3 scripts/calibrate_thrust.py --mass 0.034 --height 0.5 --out cal.yaml
    python3 scripts/calibrate_thrust.py --offline cal_samples.csv --mass 0.034 --out cal.yaml

## Bring-up checklist (props OFF first)

1. Run the five test suites (README, Tests).
2. Driver defaults: confirm `cmd_rate` is refused (warning) and flightmode is
   unchanged (`ros2 param`/cfclient shows stabModeRoll/Pitch = 1).
3. Props off: check `cf1/imu` rate (about `imu_log_hz`) and sign of each gyro
   axis by hand-rotating the drone; accel z about +9.81 at rest.
4. Owner approval to change the flight mode. Then props off, `no_fly:=true`
   or on the bench: `enable_rate_cmd:=true rate_mode_setup:=true`; confirm the
   params go to 0 on takeoff and back to 1 on land, `stop`, emergency and
   shutdown.
5. Props off, motors spinning only if safe/restrained: verify roll, pitch, yaw
   rate signs (mixer response of the motors) and that a thrust step matches the
   N -> word map.
6. Calibrate thrust (hover, position mode), supply the YAML.
7. Tethered or netted first flight: low rates, short, hand on `emergency`.
8. Check the stale-command hold and tilt-guard auto-land.

## Known gaps

- The teleop (`cf_keyboard`/`crazyflie_vicon_teleop.py`) ATTITUDE mode would
  read angles as rates if connected to a drone left in rate mode.
- Param acknowledgment and the freshness callbacks are exercised only against
  a mocked cflib; confirm with props off that `add_update_callback` fires for
  both write echoes and `request_param_update` replies on your cflib version.
