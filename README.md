## Approach: Using Ubuntu 22 and ROS2 Humble

1. Move to the source directory of your ROS workspace. Also make sure you source into ROS humble and have Ubuntu 22 OS
```bash
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=waffle_pi
    cd ros2_ws/src
```
3. Run below commands to perform teleoperation
```bash
    # Terminal 1
    colcon build --packages-select obstacle_avoidance_tb3
    source install/setup.bash
    ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage2.launch.py
    # Terminal 2
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run obstacle_avoidance_tb3 turtlebot_teleop.py
```
4. Run below commands to execute obstacle Avoidance algorithm
```bash
    # Terminal 1
    colcon build --packages-select obstacle_avoidance_tb3
    source install/setup.bash
    ros2 launch obstacle_avoidance_tb3 launch.py
```
