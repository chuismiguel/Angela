#!/bin/bash

ROS_WORKSPACE_PATH=/home/angela/angela_ws
CONTAINER_NAME="angela_ros_running"
LAUNCH_FILE="motor_launch.py"

if [ "$#" -ne 1 ]; then
    LAUNCH_COMMAND="source /ros_ws/ros_source.sh && \
      source /opt/ros/humble/setup.bash && \
      source /ros_ws/install/setup.bash && \
      cd /ros_ws/launch && \
      ros2 daemon start && \
      ros2 launch $LAUNCH_FILE"
    INTERACTIVE_MODE=""
    COMMAND_FLAG="-c "
else
    COMMAND_FLAG=""
    LAUNCH_COMMAND=""
    INTERACTIVE_MODE="-it"
fi


# Run the container with the ROS workspace mounted
docker run --init --privileged --platform linux/arm64/v8 --network=host --rm -v "$ROS_WORKSPACE_PATH:/ros_ws" --device /dev/gpiomem -v  /sys:/sys --name "$CONTAINER_NAME" $INTERACTIVE_MODE angela_ros_runtime:latest /bin/bash  $COMMAND_FLAG $LAUNCH_COMMAND

docker kill $CONTAINER_NAME
