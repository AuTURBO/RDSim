# **Introduction**

## **RDSim**

<!-- ![RDSim](assets/logo.png "RDSim Logo") -->

**RDSim** is a high-fidelity **Robo Delivery Simulator** designed for autonomous delivery systems.

- ***Realism:*** RDSim integrates advanced **SLAM, localization, planning, and control** technologies within the **Gazebo simulation** environment.
- ***Modularity:*** The simulator supports various robotic platforms and is designed to be easily extensible for different applications.
- ***Autonomy:*** Features include **MPPI-based motion planning**, **topology-based navigation**, and **GPS-IMU fusion localization** for real-world deployment.
- ***Multi-Sensor Support:*** RDSim enables sensor fusion with:
  - **3D LiDAR SLAM** (e.g., Velodyne, Ouster)
  - **IMU integration** (e.g., Xsens, MicroStrain)
  - **GPS localization** (e.g., RTK GPS)
- ***Ease of Use:*** RDSim offers **Docker-based deployment**, enabling a seamless setup for simulation and development.

Tested on **Ubuntu 22.04 / 24.04** with **ROS 2 Humble** and **Gazebo Fortress / Ignition**.

[![Build Status](assets/build.svg)](https://github.com/AuTURBO/RDSim/actions/workflows/build.yml)  [![GitHub](https://img.shields.io/badge/GitHub-Repository-black?logo=github)](https://github.com/AuTURBO/RDSim)  


---

## **Demo**

### **RDSim Overview**

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/LW87tunwvLI?si=KmcLMo9WP7M93-m2" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

---

### **SLAM Integration using GLIM and HDL Localization**


<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/ZHhsIfWeoyU?si=471Bfbetc_uCx89V" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>
---

### **Topology-Based Navigation with MPPI Control**


<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/xLx2PAmULNQ?si=WgUqUQl_z2d5qEuD" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

---

### **Outdoor Driving Simulation with GPS-IMU Fusion**


<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/lxEWh2Fm4H4?si=MYT8Ng2nI_HzzBrn" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

---

## **Features and Updates**

### ***(2024.11) Ongoing Enhancements***
- **ROS 2 control integration**
- **Segmentation of drivable areas**
- **Advanced behavior tree (BT) applications for decision-making**

### ***(2024.10) Advanced Planning and Control***
- **MPPI controller tuning**
- **Topology map navigation with Nav2**
- **GPS + IMU + HDL-based relocalization**
- **Topology Map Editor for flexible route planning**

### ***(2024.09) SLAM & Localization Improvements***
- **3D LiDAR SLAM** ([GLIM](https://github.com/koide3/glim))
- **Nav2 package configuration**
- **GLIM factor graph to PCD conversion**
- **Localization with** ([HDL localization](https://github.com/koide3/hdl_localization)) & Nav2 integration

### ***(2024.04) Initial Development***
- **Based on** [CitySim](https://github.com/osrf/citysim)
- **Basic robot control and navigation**
- **Docker container support**
- **Gazebo dynamic actor simulation** ([gazebo_sfm_plugin](https://github.com/robotics-upo/gazebo_sfm_plugin))

See more details in [Extension Modules](extensions.md) and [Demo](demo.md) pages.

---
<!-- 
## BibTeX

```bibtex
@article{yoon2024rdsim,
  author    = {Yoon, Woosung and Kim, Daewan and Liu, Lingjie and Lee, Geonhee and Kim, Minwoo and Choi, Seongjun},
  title     = {RDSim: Robo Delivery Simulator},
  year      = {2024},
  url       = {https://github.com/AuTURBO/RDSim},
} -->
