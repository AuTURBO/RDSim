FROM althack/ros2:humble-cuda-gazebo-nvidia-2024-03-01

# NVIDIA 컨테이너 런타임 설정
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} 
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics 

USER root

# 타임존 설정
ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 필수 패키지 설치 및 불필요한 파일 정리
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    binutils \
    mesa-utils \
    x-window-system \
    python3-pip \ 
    terminator \
    gedit \
    cmake \
    git \
    vim \
    tree \
    software-properties-common \
    apt-transport-https \
    wget \
    curl \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Gazebo 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    libgazebo-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 2 패키지 설치
ARG ROS_DISTRO=humble
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-spatio-temporal-voxel-layer \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-xacro \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 2 워크스페이스 설정
ARG WORKSPACE=/root/ros2_ws
ENV WORKSPACE=$WORKSPACE
RUN mkdir -p $WORKSPACE/src
WORKDIR $WORKSPACE

# ROS 2 및 Gazebo 환경 변수 설정
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=34" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:\$GAZEBO_RESOURCE_PATH" >> /root/.bashrc && \
    echo "source ~/ros2_ws/install/local_setup.bash" >> /root/.bashrc && \
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

RUN rosdep update

# GitHub 리포지토리 설치 스크립트 복사 및 실행
COPY install.sh /tmp/install.sh
RUN chmod +x /tmp/install.sh && /tmp/install.sh && rm /tmp/install.sh 

# 컨테이너 외부와의 통신을 위한 포트 공개
# WebSocket server
EXPOSE 8080  
# WebSocket communication
EXPOSE 7681  
# Gazebo master server
EXPOSE 11345 
