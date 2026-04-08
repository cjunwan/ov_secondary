#!/bin/bash

# 1. 호스트의 X 서버 접근 허용
xhost +local:docker
xhost +

# 2. 호스트 경로 설정 (필요에 따라 수정하세요)
DATASET_PATH="/home/woong8879/datasets"
WORKSPACE_PATH="/home/woong8879/microswarm_newvio"
STX_INIT_PATH="/home/woong8879/stx-init"

# 3. 컨테이너 이름 및 이미지 설정
CONTAINER_NAME="micro_ros1_v2"
IMAGE_NAME="ov_2:latest"

# 4. 기존 컨테이너가 있으면 시작하고 접속, 없으면 새로 run
if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    echo "Existing container found. Starting and attaching..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Creating new container..."
    docker run -it \
        --name $CONTAINER_NAME \
        --gpus all \
        --privileged \
        --net=host \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XAUTHORITY=/tmp/.docker.xauth \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/bus/usb:/dev/bus/usb:rw \
        -v /etc/group:/etc/group:ro \
        -v /etc/passwd:/etc/passwd:ro \
        -v $DATASET_PATH:/home/woong8879/datasets \
        -v $WORKSPACE_PATH:/home/woong8879/microswarm_newvio \
        $IMAGE_NAME /bin/bash
fi
