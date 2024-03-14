FROM althack/ros2:humble-cuda-gazebo-nvidia-2024-03-01

#nvidia-container-runtime 
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all} 
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics 

USER root 

# Set the timezone
ENV TZ Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \    
    binutils \
    mesa-utils \
    x-window-system \
    && rm -rf /var/lib/apt/lists/*

# Install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends gazebo && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 dependencies
ARG ROS_DISTRO=humble

# Prevent hash mismatch error for apt-get update
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# Install non-Python/ROS dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

# Copy and run install.sh
COPY install.sh /tmp/install.sh
RUN chmod +x /tmp/install.sh

# Set up ROS 2 workspace
ARG WORKSPACE=/root/ros2_ws
ENV WORKSPACE=$WORKSPACE
WORKDIR $WORKSPACE

# Install ROS 2 packages and dependencies
RUN apt-get update && apt-get install -y \
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
    ros-humble-twist-mux && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --allow-unauthenticated \
	python3-pip \
	terminator \
	gedit \
	cmake \
	git \
	vim \
	tree \
	software-properties-common \
	apt-transport-https \
	wget

# Set ROS2 environment variables in /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=34" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc && \
    echo "echo 'ROS2 environment set, Domain = 34'" >> /root/.bashrc

# Set ROS2 Humble and ROS aliases in /root/.bashrc
RUN echo "source ~/ros2_ws/install/local_setup.bash" >> /root/.bashrc && \
    echo "alias cw='cd ~/ros2_ws'" >> /root/.bashrc && \
    echo "alias cs='cd ~/ros2_ws/src'" >> /root/.bashrc && \
    echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbp='cd ~/ros2_ws && colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> /root/.bashrc && \
    echo "alias gb='gedit ~/.bashrc'" >> /root/.bashrc && \
    echo "alias rt='ros2 topic list'" >> /root/.bashrc && \
    echo "alias re='ros2 topic echo'" >> /root/.bashrc && \
    echo "alias rn='ros2 node list'" >> /root/.bashrc && \
    echo "alias testpub='ros2 run demo_nodes_cpp talker'" >> /root/.bashrc && \
    echo "alias testsub='ros2 run demo_nodes_cpp listener'" >> /root/.bashrc && \
    echo "source '/usr/share/gazebo/setup.sh'" >> /root/.bashrc 

# setup environment
EXPOSE 8080
EXPOSE 7681
EXPOSE 11345

CMD ["gzserver"]
