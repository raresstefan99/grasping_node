# 🦾 Grasping Node (ROS 2)

A semi-automated robotic grasping routine implemented in C++ using ROS 2 and MoveIt. The node controls a UR manipulator (simulated in CoppeliaSim) to detect, approach, grasp, move, and place objects based on visual keypoint data. The process is controlled step-by-step via keyboard flags to give full control to the user.

## 📦 Package Overview

This package:
- Subscribes to keypoint detections (from a vision system, e.g., YOLO).
- Plans and executes movements with MoveIt.
- Publishes grasp/release commands to control a simulated gripper.
- Visualizes the grasp target in RViz using markers.
- Allows user control over every action via keyboard input.

## 🛠️ Features

- Cartesian and joint-space trajectory planning
- Interactive state machine (`GraspState`)
- Flag-based execution control (`'1'` to proceed, `'c'`/`'r'` to confirm or reject a keypoint)
- Compatible with both simulation and real robots
- Visual debugging in RViz (`visualization_msgs/Marker`)

## 📋 Dependencies

Make sure you have these ROS 2 packages installed:

- `rclcpp`
- `geometry_msgs`
- `std_msgs`
- `visualization_msgs`
- `manipulators` (custom library for motion planning)

Also, this node assumes you are using:
- A UR robot (e.g., UR10e) configured via MoveIt
- A topic `/keypoint_data` publishing `geometry_msgs/Point` (you can use **detection_bag** in my repositories "https://github.com/raresstefan99/bag_detection")
- A topic `/grasp_control` to trigger gripper actions in CoppeliaSim simulation

## 🚀 Usage

### 1. Clone the Package in your workspace

```bash
cd ~/ros2_ws
git clone https://github.com/raresstefan99/grasping_node
```

### 2. Build the Package

```bash
colcon build --packages-select grasping_node --symlink-install
source install/setup.bash
```

### 3. Launch the Simulation

Make sure your robot and MoveIt configuration are running. You can launch your environment (UR + MoveIt + RViz + CoppeliaSim) with your custom launch file.

### 4. Run the Grasping Node

```bash
ros2 run grasping_node grasp_bag
```

### 5. Keyboard Commands

After launching, use the terminal to control each step of the grasp:

    h → move the robot to the home position

    s → start accepting keypoints from /keypoint_data

    c → confirm the keypoint and proceed with grasping

    r → reject and wait for a new keypoint

    1 → proceed to the next step in the grasping sequence

    q → quit

## 🧠 State Machine

Approach → Grasp → Move → Place → Home → Idle

Each state waits for the user to press '1' before continuing, allowing step-by-step control.

## 🧪 Notes

    Designed primarily for CoppeliaSim simulation.

    Adaptable to real robots by replacing the joint publishers and gripper control.

    Ensure the frame base_link matches the robot's base in RViz and simulation.

## 📝 License

This project is licensed under the Apache-2.0 License. See LICENSE for more details.

## 👤 Author

Developed by [Rares Stefan], June 2025.

If you find this useful, feel free to ⭐️ the repo and contribute!






