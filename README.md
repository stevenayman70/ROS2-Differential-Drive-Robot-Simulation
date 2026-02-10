# ROS 2 Differential Drive Robot Simulation

This repository contains a professional ROS 2 (Humble) package for the simulation and control of a differential drive mobile robot. The project demonstrates advanced robotics concepts including URDF/Xacro modeling, Gazebo physics integration, and sensor fusion.

## 🚀 Key Features
* **Modular Robot Description:** Clean Xacro files defining the physical, visual, and collision properties of the robot.
* **Gazebo Physics Integration:** Fully configured for realistic simulation with differential drive controllers and joint state publishers.
* **Sensor Suite:** Integrated support for RPLidar (laser scans), Depth Cameras, and standard RGB cameras.
* **Custom Worlds:** Includes predefined Gazebo `.world` files featuring obstacle courses and indoor environments.
* **Control Ready:** Pre-configured launch files for teleoperation via Joystick or Keyboard.

## 📂 Project Evolution
1. **Phase 1: Hardware Modeling** - Building the 3D representation and kinematics using URDF and Xacro.
2. **Phase 2: Simulation Environment** - Establishing Gazebo plugins for differential drive and sensor feedback.
3. **Phase 3: Autonomy & SLAM** - Configuring the system for SLAM (Simultaneous Localization and Mapping) and Navigation2.

## 🛠️ Tech Stack
* **Framework:** ROS 2 (Humble)
* **Simulator:** Gazebo 11
* **Visualization:** RViz2
* **Languages:** Python (Launch Scripts), XML/Xacro (Robot Modeling)

## 📂 Repository Structure

| Directory | Description |
| :--- | :--- |
| **`/description`** | URDF and Xacro files defining robot geometry and sensors. |
| **`/launch`** | Python launch scripts for Gazebo, RViz, and controller nodes. |
| **`/config`** | YAML parameter files for Twist Mux and sensor configurations. |
| **`/worlds`** | Gazebo environment files (.world) for testing. |



## 📖 How to Build and Run

### 1. Prerequisites
* Ubuntu 22.04
* ROS 2 Humble
* Gazebo and RViz2 installed

### 2. Installation & Build
```bash
# Create a workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone this repository into the src folder
# git clone <your-repo-link>

# Install dependencies and build
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```


### 3. Launching the Simulation
* **To start the robot in a Gazebo world:**
  ```bash
  ros2 launch articubot_one_main launch_sim.launch.py
  #To open RViz for visualization:
  rviz2 -d src/articubot_one_main/config/view_bot.rviz
  ```

---

## ⚖️ Part 5: License
This project is licensed under the MIT License.

---
### Author
**[Steven_Tawfik]** 

