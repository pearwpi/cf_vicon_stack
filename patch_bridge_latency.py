#!/usr/bin/env python3
"""Make vicon_receiver report REAL latency, and count dropped Vicon frames.

Run from the cf_vicon_stack root:   python3 patch_bridge_latency.py
Then:  colcon build --packages-select vicon_receiver

WHAT AND WHY
------------
1. header.stamp becomes CAPTURE time, not pull time.

   Today the bridge stamps each message with get_clock()->now() at the moment it
   pulls the frame, so every downstream consumer measures the ROS transport hop
   (~0.19 ms) and nothing else. That is why hover_log.csv's latency_ms reads 0.

   GetLatencyTotal() reports camera-to-client latency as a DURATION, so
   subtracting it from the local clock needs no clock synchronisation with the
   Vicon server. After this, `now - stamp` is a real end-to-end latency
   everywhere, and the stamp finally means what ROS consumers (tf,
   message_filters) already assume it means.

   Nothing safety-critical reads the stamp. ViconSource.age() uses local
   receive time, so MOCAP_STALE_S / MOCAP_DEAD_S are untouched. Expect the
   reported latency to jump from ~0.19 ms to a real figure, likely single-digit
   to low-teens milliseconds.

2. The frame number stops being discarded.

   get_frame() already calls GetFrameNumber() and throws the result away; GCC
   warns about it on every build. The counter is monotonic, so a jump of more
   than one is a genuinely dropped Vicon frame. This is the only way to
   distinguish frames the SYSTEM dropped from gaps this machine failed to read.

Both are reported on the console every ~5 s. No new topics, no new message
types, no new package dependencies -- deliberately, to keep the build surface
identical.
"""
import os
import re
import shutil
import subprocess
import sys
import time

HPP = "src/vicon_receiver/include/vicon_receiver/communicator.hpp"
CPP = "src/vicon_receiver/src/communicator.cpp"
for f in (HPP, CPP):
    if not os.path.exists(f):
        sys.exit(f"not found: {f}\nRun from the cf_vicon_stack root.")

# ---- verify the SDK actually exposes what we are about to call --------------
# The Output_* classes are declared in IDataStreamClientBase.h, NOT in
# DataStreamClient.h, which only declares the methods. Searching one file finds
# GetLatencyTotal() but not Output_GetLatencyTotal, which is exactly the false
# negative this check hit on 2026-08-28. Read every header in the directory.
sdk_dir = None
for arch in ("x86_64", "aarch64"):
    d = f"src/vicon_receiver/third_party/{arch}/vicon_sdk/include/vicon-datastream-sdk"
    if os.path.isdir(d):
        sdk_dir = d
        break
if sdk_dir is None:
    sys.exit("could not find the vendored vicon-datastream-sdk include directory")

headers = sorted(f for f in os.listdir(sdk_dir) if f.endswith(".h"))
h = "\n".join(open(os.path.join(sdk_dir, f), encoding="utf-8",
                   errors="replace").read() for f in headers)
sdk = os.path.join(sdk_dir, "*.h")
print(f"checking {len(headers)} header(s) in {sdk_dir}: {', '.join(headers)}")
if "GetLatencyTotal" not in h:
    sys.exit("*** this SDK has no GetLatencyTotal(). Stopping; nothing written.")

m = re.search(r"class\s+Output_GetLatencyTotal\b(.*?)\};", h, re.S)
if not m:
    sys.exit("*** could not find class Output_GetLatencyTotal. Nothing written.\n"
             "    Send Claude the output of:\n"
             f"    grep -n 'Output_GetLatencyTotal' -A12 {sdk}")
body = m.group(1)
field = None
for cand in ("Total", "Latency", "LatencyTotal"):
    if re.search(r"\b" + cand + r"\s*;", body):
        field = cand
        break
if field is None:
    sys.exit("*** Output_GetLatencyTotal exists but its member is not named\n"
             "    Total/Latency/LatencyTotal. Nothing written. Send Claude:\n"
             f"    grep -rn 'class Output_GetLatencyTotal' -A12 {sdk_dir}/")
print(f"  ok  GetLatencyTotal() present, member is '.{field}'")

fn = re.search(r"class\s+Output_GetFrameNumber\b(.*?)\};", h, re.S)
if not fn or not re.search(r"\bFrameNumber\s*;", fn.group(1)):
    sys.exit("*** Output_GetFrameNumber.FrameNumber not found. Nothing written.")
print("  ok  GetFrameNumber().FrameNumber present")

hpp, cpp = open(HPP).read(), open(CPP).read()

# ---- header: state for the two counters ------------------------------------
H_OLD = "    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;"
H_NEW = """    // Link quality, reported on the console every LOG_EVERY frames.
    // last_frame_ is the SDK's monotonic counter; a jump of more than one is a
    // frame the SYSTEM dropped, which is different from a gap this machine
    // failed to read and was previously impossible to tell apart.
    unsigned int last_frame_ = 0;
    unsigned long frames_seen_ = 0;
    unsigned long frames_dropped_ = 0;
    double latency_sum_ = 0.0;
    double latency_max_ = 0.0;
    static constexpr unsigned long LOG_EVERY = 1200;   // ~5 s at 240 Hz

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;"""

# ---- cpp: capture-time stamp + frame accounting ----------------------------
C_OLD = """    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();"""
C_NEW = """    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    // Camera-to-client latency, as a DURATION. Subtracting a duration from the
    // local clock needs no clock sync with the Vicon server, which is what
    // makes this safe to do here.
    double latency_s = 0.0;
    Output_GetLatencyTotal latency_out = vicon_client.GetLatencyTotal();
    if (latency_out.Result == Result::Success) {
        latency_s = latency_out.__FIELD__;
    }

    // Stamp with when the cameras SAW the drone, not when we got round to
    // asking. Before this, every consumer measured the ROS transport hop and
    // reported ~0 ms latency.
    const rclcpp::Time capture_time =
        this->get_clock()->now() - rclcpp::Duration::from_seconds(latency_s);

    // Frame-number continuity. The counter is monotonic, so a step of more than
    // one is a dropped frame. This value used to be fetched and discarded.
    if (frame_number.Result == Result::Success) {
        if (last_frame_ != 0 && frame_number.FrameNumber > last_frame_ + 1) {
            frames_dropped_ += (frame_number.FrameNumber - last_frame_ - 1);
        }
        last_frame_ = frame_number.FrameNumber;
    }
    frames_seen_++;
    latency_sum_ += latency_s;
    if (latency_s > latency_max_) latency_max_ = latency_s;
    if (frames_seen_ % LOG_EVERY == 0) {
        RCLCPP_INFO(this->get_logger(),
            "link: latency mean %.2f ms, max %.2f ms | dropped %lu of %lu frames (%.3f%%)",
            1000.0 * latency_sum_ / LOG_EVERY, 1000.0 * latency_max_,
            frames_dropped_, frames_seen_,
            100.0 * frames_dropped_ / (double)frames_seen_);
        latency_sum_ = 0.0;
        latency_max_ = 0.0;
    }""".replace("__FIELD__", field)

C_STAMP_OLD = """            // Use node clock to timestamp the transform
            tf_msg.header.stamp = this->get_clock()->now();"""
C_STAMP_NEW = """            // Capture time, not pull time. See get_frame()'s latency block.
            tf_msg.header.stamp = capture_time;"""

edits = [(HPP, hpp, H_OLD, H_NEW, "header counters", "frames_dropped_"),
         (CPP, cpp, C_OLD, C_NEW, "latency + frame accounting", "GetLatencyTotal"),
         (CPP, cpp, C_STAMP_OLD, C_STAMP_NEW, "capture-time stamp", "= capture_time;")]

print("\nchecking anchors...")
cur = {HPP: hpp, CPP: cpp}
todo = []
for f, _, old, new, label, marker in edits:
    if marker in cur[f]:
        print(f"  SKIP  {label} (already applied)")
        continue
    if old not in cur[f]:
        sys.exit(f"\nANCHOR NOT FOUND for '{label}' in {f}:\n{old}\n"
                 "Nothing written. Send this to Claude.")
    if cur[f].count(old) != 1:
        sys.exit(f"\nANCHOR NOT UNIQUE for '{label}' in {f}. Nothing written.")
    print(f"  ok    {label}")
    todo.append((f, old, new))

if not todo:
    sys.exit("\nAlready applied; nothing to do.")

bak = "_backup_bridge_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
for f in (HPP, CPP):
    shutil.copy2(f, os.path.join(bak, os.path.basename(f)))
print(f"\nbacked up to {bak}/")

for f, old, new in todo:
    cur[f] = cur[f].replace(old, new, 1)
for f in (HPP, CPP):
    open(f, "w").write(cur[f])
    print(f"  wrote {f}")

print("\nbuilding...")
r = subprocess.run("bash -lc 'source /opt/ros/humble/setup.bash && "
                   "colcon build --packages-select vicon_receiver'",
                   shell=True, capture_output=True, text=True)
tail = (r.stdout + r.stderr).strip().splitlines()[-20:]
print("\n".join("  " + l for l in tail))
if r.returncode != 0:
    print(f"\nBUILD FAILED. Restore with:  cp {bak}/communicator.hpp {HPP}; "
          f"cp {bak}/communicator.cpp {CPP}")
    sys.exit(1)
print("\nBuilt. Relaunch the bridge and watch for the 'link:' lines every ~5 s.")
