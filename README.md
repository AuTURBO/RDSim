## RDSim 

### Install && build

```bash
cd ~/ros2_ws/src 
git clone https://github.com/AuTURBO/RDSim.git
cd ~/ros2_ws && colcon build --symlink-install && source install/local_setup.bash
```

### Sim launch

```bash
 ros2 launch rdsim_description rdsim_gazebo.launch.py  
```