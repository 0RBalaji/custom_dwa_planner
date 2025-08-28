# Custom DWA Local Planner for ROS2 Humble

## Overview

A from-scratch implementation of Dynamic Window Approach (DWA) local planner for TurtleBot3 navigation.

## Package Structure

```bash
custom_dwa_planner /
├── config
│   └── params.yaml
├── include/custom_dwa_planner
│       └── dwa_planner.hpp
├── launch
│   └── planner.launch.py
├── src
│   ├── dwa_planner.cpp
│   └── main.cpp
├── rviz
│   └── default_view.rviz
├── package.xml
├── CMakeLists.txt
├── test.py
├── README.md
└── LICENSE
```


### Mechanism:


### Tools Used:

- **ROS 2 Humble**: Robot Operating System for building robot applications.
- **TurtleBot3**: A low-cost, open-source robot platform.
- **Gazebo Classic**: A simulation environment for testing the robot in custom worlds.
- **RViz2**: A 3D visualization tool for ROS applications.

---

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- TurtleBot3 packages installed


## Installation Steps

Install the dependencies required for this project initially, including postgresql requirement.
```bash
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-rviz2
```

Next, clone the repository and build the workspace.
```bash
mkdir -p dwa_planner/src
cd dwa_planner/src
git clone https://github.com/0RBalaji/custom_dwa_planner.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select custom_dwa_planner
source install/setup.bash
```

The Setup is completed and now, the system is ready to run.

Make sure, the turtlebot3 files are installed and are run it before, to make sure the .STL files are cached locally.

### Launch methods
```bash
echo "source ~/dwa_planner/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

#### Terminal tab 1(Main launch)

```bash
ros2 launch custom_dwa_planner planner.launch.py
```
This will launch the turtlebot3 rsp, the gazebo with the world file and spawn the robot, the rviz2 window and the custom dwa planner node.

#### Terminal tab 2(goal Command)

Run the Python script to send the goal to the node
```bash
/usr/bin/python3 ~/dwa_planner/src/custom_dwa_planner/test.py
```
This will send a goal to the custom local planner and the robot will follow the paths to reach the goal.
---

## Features

- Dynamic velocity sampling within robot constraints
- Real-time obstacle avoidance using laser scan data
- Trajectory visualization in RViz
- Configurable parameters via YAML file
- Minimal Logging for debugging

---
<!-- ## Working Demo

![]() -->

---

## Parameters

- max_linear_vel: Maximum forward velocity (0.22 m/s)
- max_angular_vel: Maximum angular velocity (2.84 rad/s)
- prediction_time: Trajectory prediction horizon (3.0 s)
- goal_tolerance: Distance tolerance to goal (0.2 m)

---

## TroubleShooting

- Ensure all dependencies are installed correctly.
- Ensure all TurtleBot3 environment variables are set
- Check that laser scan topic /scan is publishing data
- Verify odometry topic /odom is available
- Use ros2 topic list to confirm topic availability
- Check terminal logs for any error messages

---

## Future Improvements

- Improve by adaptive sampling: fewer samples when close to goal or obstacles.
- Optimize trajectory scoring for smoother paths.
- Instead of getting the current pose, get the predicted_pose for future, on the path obstacle 

---

## Acknowledgements
- [TurtleBot3](https://www.turtlebot.com/) for the robot platform.\
- [Gazebo Classic](http://gazebosim.org/) for the simulation environment.
- [RViz2](https://www.ros.org/reps/rep-0105.html) for visualization.
---

## License
This project is licensed under the Apache-2.0 License. See the [LICENSE](LICENSE) file for details