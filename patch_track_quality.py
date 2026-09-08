#!/usr/bin/env python3
"""Publish how many markers Vicon actually saw, and its own fit quality.

Run from the cf_vicon_stack root:  python3 patch_track_quality.py
Then:  colcon build --packages-select vicon_receiver

WHY
---
Findings 1.7: the 180 deg attitude flips come from PARTIAL occlusion. Two of
seven markers go dark under thrust -- three sit on the motor axes with a prop
hub directly above -- and Vicon fits the remaining five to a solution rotated
about a near-horizontal axis. It still publishes a pose. The pose is wrong.

Nothing in the stack can see that. The occlusion sentinel added earlier catches
only TOTAL loss, where the pose stops. A degraded fit looks exactly like a good
one from the outside, which is why five sessions of debugging had to infer the
marker state backwards from a yaw histogram after each flight.

The DataStream SDK has been able to answer this directly the whole time:

    GetObjectQuality(subject)              -> double Quality
    GetMarkerCount(subject)                -> unsigned int MarkerCount
    GetMarkerName(subject, i)              -> String MarkerName
    GetMarkerGlobalTranslation(subject, m) -> bool Occluded

(Field names verified against IDataStreamClientBase.h in this tree, not assumed
-- the earlier latency patch was written against the wrong header first.)

WHAT THIS ADDS
--------------
1. A companion topic per segment: <pose topic>/quality, geometry_msgs/
   Vector3Stamped, with (x, y, z) = (markers_visible, markers_expected,
   quality). Same header, so it ages with the pose.

   Vector3Stamped rather than a custom message on purpose: vicon_receiver
   generates no interfaces today, and adding rosidl generation is a build
   change with real risk for what is a diagnostic. If this earns its keep,
   promoting it to a real message is a contained follow-up.

2. A throttled warning in the bridge log whenever a subject is tracked from a
   partial marker set.

Nothing safety-critical reads the new topic yet, and no existing behaviour
changes -- this is observation first. track_monitor.py consumes it.

Idempotent and all-or-nothing: anchors are checked before anything is written,
originals go to _backup_quality_<timestamp>/. Anchors deliberately avoid every
line the latency patch touched, so this applies to a patched or unpatched tree.
"""
import os
import shutil
import subprocess
import sys
import time

HPP = "src/vicon_receiver/include/vicon_receiver/communicator.hpp"
CPP = "src/vicon_receiver/src/communicator.cpp"
for f in (HPP, CPP):
    if not os.path.exists(f):
        sys.exit("not found: %s\nRun from the cf_vicon_stack root." % f)
cur = {f: open(f).read() for f in (HPP, CPP)}
edits = []

# --- header: include + the publisher map ------------------------------------
H1_OLD = '#include "geometry_msgs/msg/transform_stamped.hpp"'
H1_NEW = ('#include "geometry_msgs/msg/transform_stamped.hpp"\n'
          '#include "geometry_msgs/msg/vector3_stamped.hpp"')
edits.append((HPP, H1_OLD, H1_NEW, "hpp: Vector3Stamped include",
              'geometry_msgs/msg/vector3_stamped.hpp'))

H2_OLD = "    std::set<std::string> pending_publishers;"
H2_NEW = """    std::set<std::string> pending_publishers;

    // Track-quality companion publishers, keyed "subject/segment" exactly like
    // pub_map. Created alongside the pose publisher so that by the time a pose
    // publisher reports ready, its quality publisher exists too.
    map<string, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>
        quality_pub_map;"""
edits.append((HPP, H2_OLD, H2_NEW, "hpp: quality_pub_map member",
              "quality_pub_map;"))

# --- cpp: create the companion publisher ------------------------------------
C1_OLD = """    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));
    pending_publishers.erase(key);"""
C1_NEW = """    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));
    // rclcpp::Node:: is required, not decoration. This class declares its own
    // create_publisher(subject, segment), and a derived-class member hides EVERY
    // base member of that name -- including the rclcpp::Node::create_publisher<T>
    // template. Unqualified, the compiler picks the non-template overload and
    // parses the '<' as less-than: "expected primary-expression before '>'".
    quality_pub_map[key] = this->rclcpp::Node::create_publisher<
        geometry_msgs::msg::Vector3Stamped>(topic_name + "/quality", 10);
    pending_publishers.erase(key);"""
edits.append((CPP, C1_OLD, C1_NEW, "cpp: create the quality publisher",
              'topic_name + "/quality"'))

# --- cpp: measure it, once per subject --------------------------------------
C2_OLD = """        // Get the number of segments for the subject
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;"""
C2_NEW = """        // Get the number of segments for the subject
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        // --- how good is this fit, really? ----------------------------------
        // A pose fitted from a partial marker set is published exactly like a
        // good one. Ask Vicon directly rather than inferring it later from the
        // yaw trace. Both calls read the frame already pulled into the client
        // buffer, so this is local work, not extra network traffic.
        double object_quality = -1.0;
        Output_GetObjectQuality quality_out = vicon_client.GetObjectQuality(subject_name);
        if (quality_out.Result == Result::Success)
        {
            object_quality = quality_out.Quality;
        }

        unsigned int markers_expected = 0;
        unsigned int markers_visible = 0;
        Output_GetMarkerCount marker_count_out = vicon_client.GetMarkerCount(subject_name);
        if (marker_count_out.Result == Result::Success)
        {
            markers_expected = marker_count_out.MarkerCount;
            for (unsigned int marker_index = 0; marker_index < markers_expected; ++marker_index)
            {
                Output_GetMarkerName marker_name_out =
                    vicon_client.GetMarkerName(subject_name, marker_index);
                if (marker_name_out.Result != Result::Success)
                {
                    continue;
                }
                Output_GetMarkerGlobalTranslation marker_out =
                    vicon_client.GetMarkerGlobalTranslation(subject_name,
                                                            marker_name_out.MarkerName);
                if (marker_out.Result == Result::Success && !marker_out.Occluded)
                {
                    ++markers_visible;
                }
            }
        }
        if (markers_expected > 0 && markers_visible < markers_expected)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "%s: %u of %u markers visible (quality %.2f) -- pose is fitted from a "
                "partial set and can settle into a rotated solution",
                subject_name.c_str(), markers_visible, markers_expected, object_quality);
        }"""
edits.append((CPP, C2_OLD, C2_NEW, "cpp: measure quality and visible markers",
              "GetObjectQuality(subject_name)"))

# --- cpp: publish it beside the pose ----------------------------------------
C3_OLD = """                        // Publish the transformed pose
                        pub.publish(global_pose_msg);"""
C3_NEW = """                        // Publish the transformed pose
                        pub.publish(global_pose_msg);

                        // Companion sample on <topic>/quality, same header so
                        // a consumer can age it exactly like the pose.
                        //   x = markers seen this frame
                        //   y = markers the template expects
                        //   z = Vicon's own fit quality, or -1 if unavailable
                        std::map<std::string, rclcpp::Publisher<
                            geometry_msgs::msg::Vector3Stamped>::SharedPtr>::iterator
                            quality_it = quality_pub_map.find(subject_name + "/" + segment_name);
                        if (quality_it != quality_pub_map.end() && quality_it->second)
                        {
                            geometry_msgs::msg::Vector3Stamped quality_msg;
                            quality_msg.header = tf_msg.header;
                            quality_msg.vector.x = static_cast<double>(markers_visible);
                            quality_msg.vector.y = static_cast<double>(markers_expected);
                            quality_msg.vector.z = object_quality;
                            quality_it->second->publish(quality_msg);
                        }"""
edits.append((CPP, C3_OLD, C3_NEW, "cpp: publish the quality companion",
              "quality_msg.vector.x"))

todo, done = [], []
for f, old, new, label, marker in edits:
    if marker in cur[f]:
        done.append(label)
    elif old in cur[f]:
        todo.append((f, old, new, label))
    else:
        sys.exit("ANCHOR NOT FOUND for '%s' in %s.\nNothing written." % (label, f))
for l in done:
    print("  already applied: %s" % l)
if not todo:
    print("\nNothing to do.")
    sys.exit(0)

bak = "_backup_quality_" + time.strftime("%Y%m%d-%H%M%S")
os.makedirs(bak, exist_ok=True)
for f in {t[0] for t in todo}:
    shutil.copy2(f, os.path.join(bak, os.path.basename(f)))
print("\nbacked up to %s/" % bak)
for f, old, new, label in todo:
    cur[f] = cur[f].replace(old, new, 1)
    print("  %s" % label)
for f in {t[0] for t in todo}:
    open(f, "w").write(cur[f])
print("\nwrote %s" % ", ".join(sorted({t[0] for t in todo})))

if shutil.which("colcon") is None:
    print("\ncolcon not on PATH -- files patched, build skipped.")
    print("On the ROS machine run:  colcon build --packages-select vicon_receiver")
    sys.exit(0)

print("\nbuilding...")
r = subprocess.run("bash -lc 'source /opt/ros/humble/setup.bash && "
                   "colcon build --packages-select vicon_receiver'",
                   shell=True, capture_output=True, text=True)
print("\n".join("  " + l for l in (r.stdout + r.stderr).strip().splitlines()[-25:]))
if r.returncode != 0:
    print("\nBUILD FAILED. Restore with:")
    print("  cp %s/communicator.hpp %s" % (bak, HPP))
    print("  cp %s/communicator.cpp %s" % (bak, CPP))
    sys.exit(1)
print("\nBuilt. Relaunch the bridge, then:")
print("  ros2 topic echo /vicon/crazyflie2/crazyflie2/quality --once")
