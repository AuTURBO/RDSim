#!/bin/bash
set -e  # Exit on error

# Install ROS 2 packages and dependencies
apt-get update
apt-get upgrade -y

rosdep update

# Clone and build RDSim
cd ~/ros2_ws/src
git clone --recursive https://github.com/AuTURBO/RDSim.git
cd ~/ros2_ws/src/RDSim/
git submodule update --remote

# build Behavior Tree
apt-get install -y libzmq3-dev libboost-dev
cd ~/ros2_ws/src/RDSim/rdsim_submodules/BehaviorTree.CPP
mkdir build
cd build
cmake ..
make
sudo make install

cd ~/ros2_ws
rosdep install --ignore-src --rosdistro humble --from-paths ./src/RDSim/rdsim_submodules/navigation2 -y

