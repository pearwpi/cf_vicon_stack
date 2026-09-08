The tests for this package live at the stack root, not here:

    ../../../test_crazyflie_ros.py

They stub rclpy and the message types so they run on a machine with no ROS
distro sourced, which is what let them be written and run at all. `colcon test`
will not pick them up; run them directly:

    cd <stack root> && python3 test_crazyflie_ros.py
