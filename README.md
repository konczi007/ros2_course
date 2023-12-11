cd ros2_ws
colcon clear
colcon build --symlink-install
ros2 launch ros2_course turtlesim_koch_launch.py
