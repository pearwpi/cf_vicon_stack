The tests for this package live in the stack's tests/ directory, not here:

    ../../../tests/test_crazyflie_ros.py

They stub rclpy and the message types, so they run without ROS. `colcon test`
does not pick them up; run them directly:

    cd <stack root> && python3 tests/test_crazyflie_ros.py
