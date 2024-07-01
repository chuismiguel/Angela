#!/bin/bash

ROS_WORKSPACE_PATH=/home/angela/angela_ws
CONTAINER_NAME="angela_ros_running"
LAUNCH_FILE="motor_launch.py"
PI_NETWORK_IFACE="wlan0"

if [ "$#" -ne 1 ]; then
  LAUNCH_COMMAND="
      source /ros_ws/ros_source.sh && \
      source /opt/ros/humble/setup.bash && \
      source /ros_ws/install/setup.bash && \
      cd /ros_ws/launch && \
      ros2 launch $LAUNCH_FILE"
  DOCKER_COMMAND="--init --privileged --platform linux/arm64/v8 --network=host --rm -v $ROS_WORKSPACE_PATH:/ros_ws --device /dev/gpiomem --device /dev/mem --device /dev/ttyAMA0 --device /dev/ttyS0 -v /sys:/sys -v /proc:/proc -v /dev:/dev --name $CONTAINER_NAME \
  angela_ros_runtime:latest /bin/bash -c \"$LAUNCH_COMMAND\""

else
DOCKER_COMMAND="--init --privileged -ti --platform linux/arm64/v8 --network=host --rm -v $ROS_WORKSPACE_PATH:/ros_ws --device /dev/gpiomem --device /dev/mem --device /dev/ttyAMA0 --device /dev/ttyS0 -v /sys:/sys -v /proc:/proc -v /dev:/dev --name $CONTAINER_NAME angela_ros_runtime:latest /bin/bash"
fi

# Run the Docker container
eval docker run $DOCKER_COMMAND

# Kill the container after execution
docker kill $CONTAINER_NAME
