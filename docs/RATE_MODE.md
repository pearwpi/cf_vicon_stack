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
- `thrust_pwm`: 0 normally. If > 0 it overrides `thrust` with a raw legacy
  thrust word (clamped to `thrust_pwm_min..max`). Bench use only.
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
| `thrust_poly`, `thrust_scale` | linear firmware map, 1.0 | newton -> thrust word |
| `thrust_pwm_min/max` | 10001 / 55000 | clamp of the thrust word |
| `enable_imu_log`, `imu_log_hz`, `imu_log_attitude` | true, 100, false | IMU stream |

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

## Revert behaviour

If, and only if, this node wrote the params, it restores
`stabModeRoll/Pitch = 1` on: landing complete, `stop` service, emergency (after
motors are cut), and node shutdown. At connect, if the firmware is found in
rate mode while `rate_mode_setup` is false (stale RAM state from a crashed
session) it is restored to 1 and an error is logged. The params live in
firmware RAM, so a power cycle also clears them. Not covered: a radio link
lost mid-flight (see "Known gaps").

## Safety behaviour

- `cmd_rate` is accepted only if: `enable_rate_cmd`, `rate_mode_setup`, the
  firmware has echoed roll/pitch/yaw = 0 (checked every tick), and state is
  FLYING. Otherwise it is refused with a throttled warning, not stored, and does
  not refresh the command watchdog.
- Stale command (no message for `command_timeout`, 0.30 s): the existing HOLD
  position fallback.
- If firmware stops confirming rate mode while rates are being flown, the driver
  never sends the numbers (they would be read as angles); it holds position.
- Rate and thrust caps as above; thrust word clamped to 10001..55000.
- Tilt guard: Vicon tilt (or onboard roll/pitch if `imu_log_attitude`) over
  `rate_tilt_limit_deg` while flying rates triggers an auto-land.
- All existing supervisor checks (Vicon loss, flight box, battery) still apply.
- A new rate command cancels the post-takeoff HOLD point; landing is not
  overridable by it.

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

- No handler for a radio link lost mid-flight: the revert cannot be sent. The
  firmware watchdog still stops motors; the params stay 0 in RAM until
  reboot or next connect (connect-time revert handles it unless
  `rate_mode_setup` is true, in which case they are simply set again).
- The teleop (`cf_keyboard`/`crazyflie_vicon_teleop.py`) ATTITUDE mode would
  read angles as rates if connected to a drone left in rate mode.
