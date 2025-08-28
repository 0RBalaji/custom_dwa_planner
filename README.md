# custom_dwa_planner
A custom implementation of the dynamic window approach based local planner for mobile robots using ROS2.


### goal command
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 2.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
