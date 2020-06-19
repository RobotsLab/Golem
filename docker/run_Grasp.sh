#!/bin/bash

TAG=$1
ROBOT=$2

xhost +local:docker

docker run -it --ulimit rtprio=99 --cap-add=sys_nice --privileged --env="DISPLAY" --network="host" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ${TAG} /bin/bash -c "cd /usr/bin && export LD_LIBRARY_PATH=/usr/bin:${LD_LIBRARY_PATH} && /usr/bin/GolemAppGrasp /usr/bin/GolemAppGrasp_Robot${ROBOT}.xml GolemAppGrasp.log stdout"
