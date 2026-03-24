# Turtlebot Autonomous Navigation & Depth-Vision Tracking 🤖

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![Python](https://img.shields.io/badge/Python-3.8-yellow) ![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

<div align="center">
  <img src="https://via.placeholder.com/600x300?text=Insert+Awesome+Robot+GIF+Here" alt="Turtlebot Navigation Demo" width="600"/>
  
  ### [🎥 Watch the full physical robot demonstration here](YOUR_YOUTUBE_OR_DRIVE_LINK_HERE)
</div>

## 📌 Overview
[cite_start]Developed for the Spring 2026 CS 424/524 Intelligent Mobile Robotics course [cite: 2, 3][cite_start], this repository contains ROS packages designed to drive a physical Turtlebot[cite: 4]. [cite_start]The project bridges the gap between simulated environments and real-world hardware, tackling Simultaneous Localization and Mapping (SLAM), autonomous waypoint navigation, and dynamic object tracking using RGB-D sensors[cite: 4, 6].

## ✨ Key Features

### Part 1: SLAM & Autonomous Navigation
* [cite_start]**Map Generation:** Utilized the `gmapping` ROS package (FastSLAM implementation) to manually map a physical lab environment[cite: 11, 19].
* [cite_start]**Waypoint Routing:** Developed a Python node (`route_navigator.py`) using `geometry_msgs/PoseStamped` to autonomously navigate the robot between three distinct coordinates (L2 -> L3 -> L1)[cite: 13, 39].
* [cite_start]**Localization:** Integrated `amcl` (Adaptive Monte Carlo Localization) for real-time pose estimation against the generated static map[cite: 33].

### Part 2: Dynamic Object Tracking (Bonus Achieved 🏆)
* [cite_start]**Computer Vision:** Created `ball_detector.py` to process camera streams and track a specific object (a red ball) while maintaining a strict 1-meter following distance[cite: 41, 53].
* [cite_start]**Depth-Only Tracking:** Engineered a robust secondary node (`bonus_detector.py`) that successfully tracks the target utilizing *only* the depth camera stream, completely bypassing the RGB sensor for environments with varying light conditions[cite: 59].

## 📂 Repository Structure
* [cite_start]`/launch` - Contains all execution files (`p2a.launch`, `p2b_nav.launch`, `p2b_vision.launch`, `p2b_bonus.launch`)[cite: 66, 76].
* [cite_start]`/script` - Cleaned, object-oriented Python source code for navigation and vision tracking[cite: 68].
* [cite_start]`/maps` - The compiled `.yaml` and `.pgm` lab map files generated during the SLAM phase[cite: 31].
* [cite_start]`/misc` - Assignment-specific text, name initials, and collaborator details[cite: 69].

## 🚀 Execution Instructions
[cite_start]The code is designed to run in a standard catkin workspace[cite: 74]. 

**For Autonomous Navigation (Part 1):**
```bash
roslaunch p2_Gallai p2a.launch
