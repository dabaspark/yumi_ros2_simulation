# YuMi ROS2 Simulation

ROS2 package for simulating the ABB YuMi robot in RViz.


## Demo
https://github.com/dabaspark/yumi_ros2_simulation/blob/main/assets/demo.mp4

https://user-images.githubusercontent.com/your-user-id/yumi_ros2_simulation/assets/demo.mp4

![YuMi ROS2 Simulation Demo](https://github.com/dabaspark/yumi_ros2_simulation/blob/main/assets/demo.mp4)



## Description
This package contains the URDF description and necessary configuration files for visualizing and simulating the ABB YuMi dual-arm robot in RViz. The package includes:
- URDF/Xacro model of the YuMi robot
- Mesh files for visualization
- Launch files for displaying in RViz
- Configuration files for robot control

## Dependencies
- Ubuntu 22.04
- ROS2 (tested on ROS 2 Humble)
- joint_state_publisher_gui
- robot_state_publisher
- rviz2
- xacro





## Installation

1. Create a workspace (if you don't have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Go to src and Clone this repository there:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/yumi_description.git
   ```

3. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install ros-humble-joint-state-publisher-gui
   sudo apt install ros-humble-robot-state-publisher
   sudo apt install ros-humble-xacro
   sudo apt install python3-catkin-pkg
   ```

4. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

5. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

6. Launch the simulation:
   ```bash
   ros2 launch yumi_description display.launch.py
   ```

The robot model should now be visible in RViz. You can use the joint_state_publisher_gui to move the robot joints.
