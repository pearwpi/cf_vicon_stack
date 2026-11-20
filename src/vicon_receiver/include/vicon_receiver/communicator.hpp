#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "vicon-datastream-sdk/DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <array>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>
#include "boost/thread.hpp"
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Transform.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;

// Main Node class
class Communicator : public rclcpp::Node
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;

    string hostname;
    unsigned int buffer_size;
    string ns_name;
    map<string, Publisher> pub_map;
    boost::mutex mutex;
    std::set<std::string> pending_publishers;

    // Track-quality companion publishers, keyed "subject/segment" exactly like
    // pub_map. Created alongside the pose publisher so that by the time a pose
    // publisher reports ready, its quality publisher exists too.
    map<string, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>
        quality_pub_map;
    
    geometry_msgs::msg::TransformStamped static_tf;
    string world_frame;
    string vicon_frame;
    vector<double> map_xyz;
    vector<double> map_rpy;
    bool map_rpy_in_degrees;

    // Link quality, reported on the console every LOG_EVERY frames.
    // last_frame_ is the SDK's monotonic counter; a jump of more than one is a
    // frame the SYSTEM dropped, which is different from a gap this machine
    // failed to read and was previously impossible to tell apart.
    unsigned int last_frame_ = 0;
    unsigned long frames_seen_ = 0;
    unsigned long frames_dropped_ = 0;
    double latency_sum_ = 0.0;
    double latency_max_ = 0.0;
    static constexpr unsigned long LOG_EVERY = 1200;   // ~5 s at 240 Hz

    // Camera poses, published as static TF frames "cam_<id>" under vicon_frame.
    // Calibration only changes when someone recalibrates, so get_frame() polls it
    // about once a second and publish_cameras() EMITS ONLY ON CHANGE. The cache
    // holds the last published [x y z qx qy qz qw] per frame (metres).
    std::map<std::string, std::array<double, 7>> camera_cache_;
    std::chrono::steady_clock::time_point next_camera_poll_{};
    void publish_cameras();

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void publish_static_transform();

public:
    Communicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the 
    // received segment data to the Publisher class.
    void get_frame();

    // functions to create a segment publisher in a new thread
    void create_publisher(const string subject_name, const string segment_name);
    void create_publisher_thread(const string subject_name, const string segment_name);
};

#endif // COMMUNICATOR_HPP
