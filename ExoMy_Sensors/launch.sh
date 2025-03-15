#!/bin/bash
#docker build -t ros2_ws .
docker run -it --rm \
    --net=host \
    --privileged \
    --device=/dev/i2c-1 \
    --device=/dev/spidev0.0 \
    --device=/dev/spidev0.1 \
    --device=/dev/ttyUSB0 \
    --env ROS_DOMAIN_ID=7 \
    -v /home/core/ExoMy_Sensors:/ExoMy_Sensors \
    ros2_ws