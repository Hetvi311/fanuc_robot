# Fanuc CRX-10iA Robot Simulation

This repository contains the Fanuc CRX-10iA robotic arm setup for ROS 2 (Jazzy) with Gazebo simulation and a custom robotic gripper. You can visualize and see tf trees in rviz and simulate the robot in Gazebo.

# Setup

## 1. Create the Package

```
mkdir -p ~/fanuc_ws/src
cd ~/fanuc_ws/src
ros2 pkg create fanuc_robot --build-type ament_python
```
---

## 2. Add Meshes and URDF

Place all robot and gripper meshes(if have any) in the meshes/ folder.

Add the URDF files in the urdf/ folder:

- fanuc_crx10.urdf.xacro → Main robot with gripper macro
- robotic_gripper.xacro → Custom gripper with prismatic fingers
- gazebo.xacro → Gazebo-specific plugins

Make sure each link has proper inertial, visual, and collision properties.

--- 

## 3. Add directory in CMakeLists.txt

For urdf, launch, meshes, rviz, config folders, you need to give the directory so that it can launch.

```
install(DIRECTORY
  urdf
  meshes
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

--- 

## 4. Launch File

Add display.launch.xml in the /launch folder 

- Visualize robot in rviz
- To see TF frames

Launch display.launch.xml
```
ros2 launch fanuc_robot display.launch.xml
```
---

## 5. Gazebo Simulation

The launch/gazebo.launch.py file:

- Sets GZ_SIM_RESOURCE_PATH
- Runs robot_state_publisher for TF and joint states
- Launches Gazebo with the empty world
- Spawns the robot and gripper
```
ros2 launch fanuc_robot gazebo.launch.py
```
