# Autonomous Warehouse Inventory Robot (ROS 2)

This project implements an autonomous mobile robot that explores an unknown
warehouse environment, navigates to shelves, and performs real-time inventory
detection and counting using computer vision and autonomous navigation.

The robot uses SLAM for mapping, Nav2 for path planning, and YOLO-based object
detection to identify and count items on shelves.

## Demo
Working Video: https://www.youtube.com/watch?v=7wsWYUg1ylI  
Detailed Report: [Project Report](docs/project_report.pdf)

## Key Features
- Autonomous exploration of unknown environments
- SLAM-based mapping and localization
- Frontier-based exploration
- Nav2 global and local path planning
- YOLO-based object detection and counting
- Real-time inventory tracking in simulation

## Tech Stack
- ROS 2 (Humble)
- Gazebo Simulation
- Nav2 Navigation Stack
- SLAM Toolbox
- YOLOv5 (TFLite)
- OpenCV
- Python

## How the System Works
1. Robot spawns in a simulated warehouse environment
2. SLAM builds an occupancy grid map
3. Exploration logic selects unexplored frontiers
4. Nav2 plans safe paths to goals
5. Robot aligns with shelves
6. Camera feed is processed using YOLO
7. Objects are detected and counted
8. Inventory data is published to ROS topics

## Running the Project (Simulation)

This repository contains a ROS 2 Python package intended to be built inside
a ROS 2 workspace.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/<repo-name>
cd ..
colcon build
source install/setup.bash
ros2 run <package_name> b3rb_ros_warehouse

## Results
- Successful autonomous exploration and navigation in simulation
- Accurate object detection and counting from camera feed
- Reliable inventory data generation using vision-based perception
- Stable integration of SLAM, Nav2, and vision pipeline in ROS 2

---

## My Contribution
I independently worked on the design, implementation, and integration of the
core robotics pipeline for this project, including:

- Designing the autonomous exploration and navigation flow
- Integrating SLAM Toolbox with Nav2 for mapping and localization
- Implementing frontier-based exploration logic
- Developing the vision pipeline for object detection and inventory counting
- Integrating YOLO inference with ROS 2 nodes
- Debugging navigation failures and tuning parameters in simulation
- End-to-end testing and validation in Gazebo

This repository is maintained and documented by me to showcase my individual
understanding and implementation of autonomous robotics systems.

---

## Limitations
- Implemented and tested only in simulation
- Object detection limited to trained YOLO classes
- Navigation performance depends on SLAM and Nav2 parameter tuning
- Performance may differ on real hardware due to sensor noise and dynamics

---

## Future Work
- Pick-and-place manipulation using robotic arm
- Deployment on real mobile robot hardware
- Semantic mapping with inventory database integration
- Multi-robot coordination for faster warehouse inventory

---

## Acknowledgement
Developed as part of an academic robotics project.


