# RDSim: Robo Delivery Simulator

### [Project Page](https://auturbo.github.io/RDSim) | [Video](https://www.youtube.com/watch?si=KmcLMo9WP7M93-m2&v=LW87tunwvLI&feature=youtu.be)

<b>About:</b>
*RDSim is a Robo Delivery Simulator developed for autonomous delivery systems. It integrates state-of-the-art SLAM, localization, planning, and control technologies within the Gazebo simulation environment. Designed as a comprehensive solution, RDSim supports robot control, environment simulation, and robust navigation capabilities.*

<div style="display: flex; justify-content: center;">
  <img src="./documents/small_sim_world.png" alt="Image 1" width="200" style="margin-right: 10px;">
  <img src="./documents/glim_result.png" alt="Image 2" width="200" style="margin-left: 10px;">
  <img src="./documents/nav2.png" alt="Image 3" width="200" style="margin-left: 10px;">
</div>

## Environment Settings
There are two ways to execute: 'local' or 'docker'


**RDSim clone**

First of all, we need to clone this project before that.

```bash
$ cd ~/ros2_ws/src
$ git clone --recursive https://github.com/AuTURBO/RDSim.git
$ cd ~/ros2_ws/src/RDSim/ && git submodule update --remote
```

### i) local: Install && build

**Requirements**
- [ROS 2 humble](https://docs.ros.org/en/humble/index.html)
- [gazebo 11](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)


**Setting GAZEBO_RESOURCE_PATH**
```sh
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc
source ~/.bashrc
```

**Install dependency**
```bash
$ sudo apt-get update && sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-interactive-marker-twist-server \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joy \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-joy \
    ros-humble-twist-mux \
    libgazebo-dev \
    ros-humble-spatio-temporal-voxel-layer \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-rclcpp-components \
    ros-humble-xacro* \
    tmux \
    tmuxp \
    && echo 'alias start_rdsim="cd ~/ros2_ws/src/RDSim/rdsim_launcher && tmuxp load rdsim_launcher.yaml"' >> ~/.bashrc \
    && echo 'alias end="tmux kill-session && killgazebo"' >> ~/.bashrc \
    && source ~/.bashrc
```

**RDSim build**
```bash
$ cd ~/ros2_ws && rosdep install --ignore-src --rosdistro humble --from-paths ./src/RDSim/rdsim_submodules/navigation2
$ colcon build --symlink-install && source install/local_setup.bash
```

### ii) docker

> Docker environment tested on Ubuntu 22.04, nvidia
>

```bash
# in rdsim main directory
cd ~/ros2_ws/src/RDSim/docker && ./run_command.sh
```

## Execute the RDSim
### All launch

```bash
start_rdsim
```

### All Kill
```bash
end
```



### Launch the Gazebo world

> Launch the only Gazebo world

```bash
ros2 launch rdsim_gazebo rdsim_gazebo_world.launch.py
```

![alt text](documents/gazebo_world.png)

---

### Loading the robot model into the GAZEBO world

```bash
ros2 launch rdsim_description rdsim_gazebo.launch.py
```

![alt text](documents/robot_model.png)

### Teleoperate the robot

> Executing the teleoperation node to control the robot via keyboard input

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Navigate the outdoor robot in the GAZEBO world


> The system supports launching localization nodes (VSLAM, EKF) and the navigation node (NAV2) for outdoor environments.


```bash
ros2 launch rdsim_gazebo rdsim_gps_navigation.launch.py
```

![alt text](documents/navigation.png)

> Navigation can detect 3D obstacles, such as trees, using a 3D LiDAR sensor and a spatio-temporal voxel layer for precise obstacle avoidance.

![alt text](documents/3d_obstacles_detection.png)

> This navigation module includes a new topology map server that supports predefined routing plans for efficient delivery in the GAZEBO simulation environment. The topology map server is implemented as a behavior, enabling the use of behavior trees for flexible and adaptive decision-making. Additionally, the behavior tree can be visualized using Groot for better understanding and debugging.

<div style="display: flex; justify-content: center;">
  <img src="documents/topology_route.png" alt="Image 1" width="300" style="margin-right: 10px;">
  <img src="documents/behavior_tree.png" alt="Image 2" width="300" style="margin-left: 10px;">
</div>

> The localization framework is based on pose estimation using the robot_localization package. It integrates data from various sensors, including:
    > - VSLAM (HDL Localization) module
    > - GPS sensor
    > - Wheel odometry
    > - IMU sensor

![alt text](documents/robot_localization.png)
_* The box represented in orange is used_
