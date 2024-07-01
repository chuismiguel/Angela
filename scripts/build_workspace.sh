# Variables

ROS_WORKSPACE_PATH=$1
TARGET_IP=$2
TARGET_USER=$3
TARGET_PASS=$4
TARGET_DIR=$5

if [ -z "$ROS_WORKSPACE_PATH" ] || [ -z "$TARGET_IP" ] || [ -z "$TARGET_USER" ] || [ -z "$TARGET_PASS" ] || [ -z "$TARGET_DIR" ]; then
  echo "Usage: $0 <ros_workspace_path> <target_ip> <target_user> <target_pass> <target_directory>"
  exit 1
fi

# Container name
CONTAINER_NAME="ros_docker_build"


# Run the container with the ROS workspace mounted
docker run --rm -v "$ROS_WORKSPACE_PATH:/ros_ws" amd64/ros:humble-ros-base /bin/bash -c "
  cd /ros_ws &&
  colcon build
"

chmod u+x scripts/angela_run.sh
cp scripts/ros_source.sh angela_ws/ros_source.sh
sshpass -p $TARGET_PASS rsync -av --progress Dockerfile angela_ws scripts/angela_run.sh $TARGET_USER@$TARGET_IP:$TARGET_DIR

# Clean up
rm -rf angela_ws/ros_source.sh

echo "Build output copied to $TARGET_USER@$TARGET_IP:$TARGET_DIR"