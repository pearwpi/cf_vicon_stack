# Lab operations

Rules for the flight volume. These are short on purpose so that they get read.

---

## 1. Who may fly

- **Never fly alone.** Two people minimum: one flying, one watching, and the
  watcher's only job is to call a stop.
- The flyer keeps a hand on the kill and eyes on the drone, not on the screen.
  If you need to read a plot, land first.
- A supervisor signs off your first flight. After that you book the slot
  yourself.

## 2. Booking

One Vicon system, one volume, one drone in the air at a time. Book a slot; do
not "just grab ten minutes" while someone else is set up — their calibration
and your drone in the volume are not compatible.

Bring your own charged batteries. The lab's are for the lab's drones.

## 3. Before the drone leaves the bench

- `python3 preflight.py --topic /vicon/<object>/<object>` — **exit 0 or no
  flight.**
- `ROS_DOMAIN_ID` set to your team number. Not zero. Check it in the container
  banner, not from memory.
- Props spin freely, no cracks, correct handedness, convex side up.
- Net up and closed.
- First run of anything new goes with `no_fly:=true`.

## 4. In the air

- Everyone in the room knows a flight is starting. Say it out loud.
- Nobody reaches into the net while the motors are live. Not to catch it, not
  to move a gate, not for a second.
- Land on any of: a guard message you do not understand, a noise change, a new
  vibration, the drone doing something you did not ask for.
- If the drone is loose and heading somewhere it should not be, **kill the
  motors.** A drone that falls two metres is cheaper than any other outcome.

## 5. Batteries

- Charge on a non-flammable surface, attended, in the designated area.
- Swollen, punctured, or hot after a crash → disposal bin, and tell the lab
  manager. Do not charge it "just to check".
- Log the pack if it was in a crash. Damage is often invisible.

## 6. After a crash

1. Power off before you pick it up.
2. Inspect: props, motor mounts, battery, frame, deck connectors.
3. Anything bent, cracked or loose gets replaced before the next flight.
4. Log it — what happened, what the drone was doing, what you changed last.
   The log is how we tell a bad night from a pattern.

## 7. Spares and damage

Consumables — propellers, motor mounts, batteries — are stocked and free.
Take what you need and tell someone when a bin runs low.

Damage from ordinary flying is expected and is not held against you. Damage
from flying without a preflight, without a net, or without a second person is
a different conversation. Report breakages the same day; a quietly returned
drone with a cracked mount is the one that hurts the next team.

## 8. Shutting down

- Batteries off the charger and out of the drone.
- Drone back in its box, props checked.
- Vicon left as you found it — do not delete other teams' objects.
- Your run logs copied off the lab machine. It gets wiped.

---

## Escalate immediately

- Any fire, smoke, or a battery that gets hot.
- Any injury, however minor.
- A drone that left the net.
- Anything you did that you are not sure was safe. Saying so early is always
  the right call and is never held against you.
