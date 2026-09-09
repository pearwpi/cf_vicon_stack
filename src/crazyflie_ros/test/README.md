The tests for this package live in the stack's tests/ directory, not here:

    ../../../tests/test_crazyflie_ros.py

They stub rclpy and the message types so they run on a machine with no ROS
distro sourced, which is what let them be written and run at all. `colcon test`
will not pick them up; run them directly:

    cd <stack root> && python3 tests/test_crazyflie_ros.py
