# Body-rate + collective-thrust (CTBR) command for crazyflie_server's cmd_rate.
# NOT a Crazyswarm2 message: local to this stack. See docs/RATE_MODE.md.
#
# Frame: Crazyflie BODY frame, FLU, right-handed (x forward, y left, z up).
# Units are SI throughout -- unlike the upstream messages, nothing is in
# degrees and nothing is sign-flipped by the caller; the driver converts.

std_msgs/Header header
float32 roll_rate    # rad/s about body +x. Positive = right side goes DOWN.
float32 pitch_rate   # rad/s about body +y. Positive = nose goes DOWN.
float32 yaw_rate     # rad/s about body +z. Positive = counter-clockwise seen from above.
float32 thrust       # COLLECTIVE thrust, newtons (sum of all four rotors), along body +z.

# Raw override. 0 = use `thrust` (normal). >0 = send this legacy thrust word
# (10001..55000 after clamping) and IGNORE `thrust`. For bench tests and the
# calibration cross-check only; a policy should always send newtons.
uint16 thrust_pwm
