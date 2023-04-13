---
marp: true
author: Khaled Gabr
theme: custom-theme
footer: ![width:200px](roboicscornerlogo.jpeg)

---


# Learn Linux and Command Line :rotating_light:

- Install Linux
- Understand Linux file system
- Basic Commands Lines

---

# Learn Shell Tools and Scripting :rotating_light:

- What is the shell?
- Using the shell
- Navigating in the shell
- Shell Scripting
- Shell Tools
  - Finding how to use commands
  - Finding files
  - Finding code
  - Finding shell commands

---

# Editors (Vim) :rotating_light:

- Which editor to learn?
  - Vim
- Philosophy of Vim
- Modal editing
- Basics Vim
  - Inserting text
  - Buffers, tabs, and windows
  - Command-line


---

# Version Control (Git) :rotating_light:

- Git’s data model
  - Snapshots
  - Modeling history
  - Data model, as pseudocode
  - Repositories
- Git command-line interface
  - Basics
  - Branching and merging
  - Remotes
  - Undo

---

# Learn C++ :rotating_light:

Learn how to develop, compile, and execute C++ programs.

- C++ Foundations

  - Data Types
  - Variables
  - Operators
  - Control Structures
    - IF conditions
    - Loops
      - For
      - while
  - Functions
  - Arrays

---

# Learn C++ :rotating_light:

Learn to control static and dynamic memory in C++.

- Memory Management
  
  - Pointers
  - Reference
  - Heap

---

# Learn C++ :rotating_light:

Learn to build classes, interfaces, and generic templates to create an object-oriented C++ program.

- Object-Oriented Programming (OOP)
  - Encapsulation
  - Inheritance
  - Polymorphism

- Templates

---

# Introduction to Robotics :rotating_light:

Learning the essential elements of robotics is a great starting point for anyone interested in pursuing a career in robotics.

- Definition and brief history of robotics
- Types of robots and their applications
- Anatomy of a robot: actuators, sensors, controllers, and effectors
- Introduction to programming languages used in robotics

---

# Introduction to ROS :rotating_light:

Learning ROS (Robot Operating System) can add significant value to your career in robotics or related fields.

- Overview of ROS
- ROS architecture and components
- ROS installation and setup
- Basic ROS commands and tools

# ROS Messages and Topics :rotating_light:

- ROS messages and message types
- Creating custom messages
- ROS topics and publishers/subscribers
- Using rostopic and rqt tools

---

# ROS Services and Actions :rotating_light:

- ROS services and service types
- Creating custom services
- ROS actions and action servers/clients
- Using rosservice and rqt tools

# ROS Nodes and Launch Files :rotating_light:

- ROS nodes and nodelets
- Creating custom nodes
- ROS launch files and launch parameters
- Using roslaunch and rqt tools
- ROS Bag

---

# ROS Packages and Catkin :rotating_light:

- ROS packages and package structure
- Creating custom packages
- Using catkin build and catkin_make
- Working with dependencies and third-party libraries

**By the end of this course, students will have a strong understanding of the ROS ecosystem and be able to develop and debug ROS applications for various robotics projects. They will have experience with the most commonly used ROS packages and tools, as well as an understanding of best practices for ROS development.**

---

# Gazebo :rotating_light:

Learning how to simulate robotic environments with Gazebo is a valuable skill for roboticists, as it allows for testing and development of robotic systems without the need for physical hardware.

- Introduction to ROS and Gazebo

  - Overview of ROS and Gazebo and their benefits for robotics development
  - Installation and setup of ROS and Gazebo
  - Creating a ROS workspace and project structure

- Building robot models in Gazebo
  - Gazebo model structure and file formats
  - Importing 3D models into Gazebo
  - Creating custom robot models in Gazebo using plugins

---

- Controlling robots with ROS
  - ROS topics, messages, and services
  - Building ROS nodes and publishers/subscribers
  - Controlling robot movements using ROS

- Simulating robot behavior in Gazebo
  - Writing Gazebo plugins to control robot behavior
  - Simulating sensors and their data using Gazebo plugins
  - Visualizing robot behavior and sensor data in Gazebo

---

# URDF (Unified Robot Description Format) :rotating_light:

 is an XML format used to describe robots and their components in ROS (Robot Operating System).

- Introduction to URDF and ROS
  - Overview of URDF and its role in ROS
  - Benefits of creating custom robots using URDF

- Creating a robot description in URDF
  - URDF file format and syntax
  - Basic robot components: links and joints
  - Defining geometry and visuals for robot components

- Advanced robot components in URDF
  - Adding sensors(Lidar- Camera- IMU,..) to the robot model
  - Using plugins to customize robot behavior and functionality

---

- Integration with other ROS packages and tools

  - Using RViz for robot visualization and debugging
  - Building custom ROS nodes to control the robot

---

# mobile robot kinematics :rotating_light:

- Introduction to Mobile Robot Kinematics
  - Definition of kinematics and its importance in mobile robotics
  - Types of mobile robots and their kinematic properties
  - Overview of coordinate frames, transforms, and transformations

- Differential Drive Kinematics
  - Differential drive robots and their kinematics
  - Deriving the kinematic model for a differential drive robot
  - Control of differential drive robots using kinematics

- Mobile Robot Localization
  - Overview of mobile robot localization
  - Sensor-based localization techniques, including odometry and wheel encoders

---

# Transformations and Frames(TF) :rotating_light:

Transformation and frames are fundamental concepts in the Robot Operating System (ROS). In ROS, a frame is a coordinate system that represents the position and orientation of a physical object, such as a robot, sensor, or camera. Transformations describe the relationship between different frames in the environment, allowing us to determine the position and orientation of objects relative to one another.

- Frames and Transformations
- Transformation Types
  - Translation
  - Rotation
- Transformation Matrix
- TF ROS

---

# Localization :rotating_light:

Learn how Kalman filter(KF) and Monte Carlo Localization (MCL) can be used to estimate noisy sensor readings, and how to estimate a robot’s position relative to a known map of the environment

# KF 

- Introduction to Kalman Filters
- Linear Systems and State Space Models
- The Prediction Step
- The Update Step
- Nonlinear Systems and Extended Kalman Filters
- Robot Loclization and robot_robot_pose ROS Packages

---

# Localization :rotating_light:

# MCL

- Introduction to Monte Carlo Localization
- What's MCL?
- Particle Filters
- Bayes Filtering
- How MCL Algorithm works?
- AMCL Package with ROS

---

# Mapping and SLAM :rotating_light:

Simultaneous Localization and Mapping (SLAM) is a technique used in robotics to map an unknown environment while simultaneously estimating the robot's pose within that environment.

- Introduction to Mapping & SLAM
- Occupancy Grid Mapping
- Grid-Based FastSLAM(Gmapping)
- GraphSLAM(Cartographer)
- SLAM integration with ROS
  - Creating a ROS package for mapping
  - create 2D map using Laser and Odometry
  - save the map
  
---

# Path Planning and Navigation :rotating_light:

Path planning and navigation are critical components of autonomous robots that allow them to move around their environment and perform tasks. There are various path planning and navigation algorithms that can be used.

- Introduction to Path Planning and Navigation
  - What is path planning and navigation?
  - Importance of path planning and navigation in robotics
  - Types of environments robots navigate in

---

# Path Planning and Navigation :rotating_light:

- Path Planning Algorithms
  - Breadth First Search(BFS) Algorithm
  - Bepth First Search(DFS) Algorithm
  - Dijkstra's Algorithm
  - A* Algorithm
  - RRT (Rapidly-exploring Random Tree) Algorithm
  - Comparison of different path planning algorithms

- Integration of SLAM and navigation
  - Navigation Integration
  - Benefits of SLAM and navigation integration

---

# Path Planning and Navigation :rotating_light:

- Move_Base ROS Package
  - Local and Global Planner
  - Local and Global Costmap
  - Recovery Behaivor
