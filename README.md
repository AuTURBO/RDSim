# RDSim: Robo Delivery Simulator

![small_sim_world](./documents/small_sim_world.png)
![glim_result](./documents/glim_result.png)
![nav2](./documents/nav2.png)

## Environment Settings
### i) local: Install && build

* Setting GAZEBO_RESOURCE_PATH
```sh
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc
source ~/.bashrc
```

* RDSim build
```bash
$ cd ~/ros2_ws/src 
$ git clone --recursive https://github.com/AuTURBO/RDSim.git
$ sudo rosdep install --ignore-src --rosdistro humble --from-paths ./src/RDSim/rdsim_submodules/navigation2
$ cd ~/ros2_ws/src/RDSim/gazebo_sfm_plugin/lightsfm && make && sudo make install
$ cd ~/ros2_ws && colcon build --symlink-install && source install/local_setup.bash
```

* Install dependency
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

### ii) docker

> Docker environment tested on Ubuntu 22.04, nvidia
> 

```bash
# in rdsim main directory
cd ~/ros2_ws/src/RDSim/docker && ./run_command.sh 
```

## Execute RDSim
### All launch 

```bash
start_rdsim
```

### All Down
```bash
end
```



### Gazebo world launch

> Gazebo 맵만 실행시킬 경우
> 

```bash
ros2 launch rdsim_gazebo rdsim_gazebo_world.launch.py  
```

### Robot Display launch 

> Gazebo 없이 로봇의 tf를 확인하고 싶을 경우
> 

```bash
ros2 launch rdsim_description rdsim_description.launch.py 
```


---

### Sim launch

```bash
ros2 launch rdsim_description rdsim_gazebo.launch.py 
```

### teleop cmd 

> cmd_vel을 통해 제어하기 때문에 다음 명령어를 통해 제어할 수 있습니다.
> 

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Execute everything with a single lunch file

```bash
ros2 launch rdsim_gazebo rdsim_navigation.launch.py
```
