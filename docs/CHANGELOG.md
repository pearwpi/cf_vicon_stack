# Changelog

How this tree got to its present shape. Nothing here is needed to use the
stack -- it is the record of what was changed and, more usefully, what was
deliberately left alone and why.

## Compaction pass (2026-08-26)

14 files / 6,463 lines -> 13 files / 6,110 lines. The line count moved less than
the structure did, and the structure is what matters here:

* **Three copies of shared logic collapsed into one.** `Teleop.supervise` was
  still a private 73-line copy of the supervisor while the ROS driver called
  `cf_core.supervise`; `clamp_alt` and the leash in `_apply_limits` were
  duplicated the same way. All three are now thin adapters over `cf_core`. This
  was a real defect, not tidying: the two front ends could have disagreed about
  when to land.
* **`vicon_pose_subscriber.py` merged into `vicon_probe.py`** (385 -> 207 lines,
  one file fewer). Link health and rigid-body health always get diagnosed
  together, so they now come from one capture. The old file is in `_to_delete/`
  because the shell used to do this cannot remove files; delete that folder.
* **One `vicon_capture()` in `cf_core`** replaces four private copies of rclpy
  subscribe/spin/shutdown. It imports rclpy inside the function, so `cf_core`
  still imports on a machine with no ROS.
* **Prose trimmed to one-liners** where it was multi-paragraph. Every safety
  constant still carries one dense line saying which failure it was written
  for; those are the lines that stop someone re-introducing the flip latch.

Deliberately NOT done: merging the four test suites into one file, which would
be the biggest remaining file-count win. `test_flip_guard.py` monkeypatches the
global `time.time` to control the clock, and since `teleop.time` IS the stdlib
module, that patch would freeze time for every other suite in the process. They
stay four processes on purpose.

## Changes from the originals

| File | Change |
|---|---|
| `vicon_probe.py` | dead `/home/manoj/...` default output -> beside the script |
| `test_tilt_origin_check.py` | dead `sys.path` entry and `CAP` capture path -> the script's own directory |
| `test_flip_guard.py` | dead `sys.path` entry -> the script's own directory |
| `crazyflie_vicon_teleop.py` | constants, `Pose`, `Bounds`, `State`, `Mode`, the flip filter and the keyboard backends moved into `cf_core.py` / `cf_keyboard.py` and imported back. No behaviour change; the 34-check flip-guard suite is the regression net and passes. 2367 -> 2096 lines. |

Fixing `CAP` recovered three checks that had been silently skipped: the
end-to-end replay of the 2026-08-12 capture, which independently reproduces the
CAUSE A template-misalignment verdict from the raw data.

`vicon_probe.py` still writes a fixed filename, so a second run overwrites the
first. Pass a name to keep a before/after pair: `python3 vicon_probe.py 30 before.csv`.
