# Turtlebot Autonomous Navigation & Depth-Vision Tracking 🤖

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![Python](https://img.shields.io/badge/Python-3.8-yellow) ![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

<div align="center">
  <video src="./misc/Color_Tracking_gif.mov" autoplay loop muted playsinline width="600"></video>
  
  ### [🎥 Watch the full physical robot demonstration here](https://drive.google.com/drive/folders/1YhiN6csOkUlHZVuiley6NDc9Yqo7oMBc?usp=sharing)
</div>

## 📌 Overview
Developed for the Spring 2026 CS 424/524 Intelligent Mobile Robotics course, this repository contains ROS packages designed to drive a physical Turtlebot. The project bridges the gap between simulated environments and real-world hardware, tackling Simultaneous Localization and Mapping (SLAM), autonomous waypoint navigation, and dynamic object tracking using RGB-D sensors.

## ✨ Key Features

### Part 1: SLAM & Autonomous Navigation
* **Map Generation:** Utilized the `gmapping` ROS package (FastSLAM implementation) to manually map a physical lab environment.
* **Waypoint Routing:** Developed a Python node (`route_navigator.py`) using `geometry_msgs/PoseStamped` to autonomously navigate the robot between three distinct coordinates (L2 -> L3 -> L1).
* **Localization:** Integrated `amcl` (Adaptive Monte Carlo Localization) for real-time pose estimation against the generated static map.

### Part 2: Dynamic Object Tracking (Bonus Achieved 🏆)
* **Computer Vision:** Created `ball_detector.py` to process camera streams and track a specific object (a red ball) while maintaining a strict 1-meter following distance.
* **Depth-Only Tracking:** Engineered a robust secondary node (`bonus_detector.py`) that successfully tracks the target utilizing *only* the depth camera stream, completely bypassing the RGB sensor for environments with varying light conditions.

## 📂 Repository Structure
* `/launch` - Contains all execution files (`p2a.launch`, `p2b_nav.launch`, `p2b_vision.launch`, `p2b_bonus.launch`).
* `/script` - Cleaned, object-oriented Python source code for navigation and vision tracking.
* `/maps` - The compiled `.yaml` and `.pgm` lab map files generated during the SLAM phase.
* `/misc` - Assignment-specific text, name initials, and collaborator details.

## 🚀 Execution Instructions
The code is designed to run in a standard catkin workspace. 

**For Autonomous Navigation (Part 1):**
```bash
roslaunch p2_Gallai p2a.launch
