# 🦾 Grasping Node (ROS 2)

An automated robotic grasping routine implemented in C++ using ROS 2 and MoveIt. The node controls a UR manipulator (simulated in CoppeliaSim) to detect, approach, grasp, move, and place objects based on visual keypoint data. 
It's also possible to control the process with a semi-automated control by the user.
The manipulator is positioned on an omnidirectional mobile base (like the Neobotix MPO-500).

The routine consists of:
- manipulator in scanning position
- 360-degree rotation of the mobile base
- identification of the bags around it
- calculation of the distance to the closest
- I approach the bag in steps (to allow a visual check of the keypoint and avoid false positives) up to a certain desired distance
- calculation and execution of the grasping trajectory if the presence of the bag is confirmed; otherwise, the search for the bags resumes
- grasping the bag and moving it to a point B
- I place the bag down and return to the scanning position
- return to the center of the room to resume scanning the bags


## 📦 Package Overview

This package:
- Subscribes to keypoint detections (from a vision system, e.g., YOLO).
- Plans and executes movements with MoveIt.
- Publishes grasp/release commands to control a simulated gripper.
- Visualizes the grasp target in RViz using markers.


## 📋 Dependencies

Make sure you have these ROS 2 packages installed:

- `rclcpp`
- `geometry_msgs`
- `ament_index_cpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `moveit`
- `manipulators` (custom library for motion planning)

Also, this node assumes you are using:
- A UR robot (e.g., UR5e) configured via MoveIt
- A topic `/keypoint_data` publishing `geometry_msgs/Point` (you can use **detection_bag** in my repositories "https://github.com/raresstefan99/bag_detection")
- A topic `/base_pose2d` publishing `geometry_msgs::msg::Pose2D` by base odometry
- A topic `/joint_states` to control if joints are reached

The node publish the following topic: 
- `grasp_control`  publish `std_msgs::msg::String` for Coppelia simulation
- `keypoint_marker` publish `visualization_msgs::msg::Marker` for debug marker in Rviz
- `cmd_vel` publish `geometry_msgs::msg::Twist` to control the mpo-500



## 🚀 Usage

### 1. Clone the Package in your workspace

```bash
cd ~/ros2_ws
git clone https://github.com/raresstefan99/grasping_node
```

### 2. Build the Package

```bash
colcon build --symlink-install --packages-select grasping_node
source install/setup.bash
```

### 3. Launch the Simulation

Make sure your robot and MoveIt configuration are running. You can launch your environment (UR + MoveIt + RViz + CoppeliaSim) with your custom launch file.

### 4. Launch the Grasping Node

```bash
ros2 launch grasping_node auto_grasp_bag
```
Or if you want to use a semi-automated process, use:

```bash
ros2 run grasping_node grasp_bag
```

The menu contains few commands:

- Go to Home position
- Go to Scan position
- Start state machine
- Stop state machine
- Advance one step

## 🧠 State Machine

First state machine: Idle → InitialPose → ScanEnvironment → ApproachTarget → End

Second state machine: Approach → Grasp → Move → Place → Home → Idle


## 🧪 Notes

    Designed primarily for CoppeliaSim simulation.

    Adaptable to real robots.

    Ensure the frame base_link matches the robot's base in RViz and simulation.

## 📝 License

This project is licensed under the Apache-2.0 License. See LICENSE for more details.

## 👤 Author

Developed by [Rares Stefan], June 2025.

If you find this useful, feel free to ⭐️ the repo and contribute!






