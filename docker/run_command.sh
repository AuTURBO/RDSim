docker build -t rdsim:v1.0 .

docker run -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /home/$USER/docker_mounted/rdsim/:/root/mounted_folder/ \
    --hostname $USER \
    --network host \
    --gpus all \
    --name rdsim \
    --ipc=host \
    rdsim:v1.0 bash