#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

if [ "$#" -lt 3 ]; then
  echo "Usage: $0 <container_name> <image_name:tag> <ros_domain_id> [dataset_path]"
  echo "  container_name : e.g. wlo1, wlo2"
  echo "  image_name:tag : e.g. tofslam:humble"
  echo "  ros_domain_id  : e.g. 1, 2 (must differ per container)"
  echo "  dataset_path   : (optional) host path to dataset directory"
  exit 1
fi

CONTAINER_NAME="$1"
IMAGE_NAME="$2"
ROS_DOMAIN_ID="$3"
DATASET_PATH="${4:-}"

VOLUME_ARGS=(
  --volume="$PROJECT_DIR/src:/root/ros2_ws/src/"
  --volume="$PROJECT_DIR/data/:/root/data"
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw
)

if [ -n "$DATASET_PATH" ]; then
  VOLUME_ARGS+=(--volume="$DATASET_PATH:/root/dataset:ro")
fi

docker run --privileged -it \
  "${VOLUME_ARGS[@]}" \
  --net=host \
  --ipc=host \
  --shm-size=4gb \
  --name="$CONTAINER_NAME" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="ROS_LOCALHOST_ONLY=1" \
  --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
  "$IMAGE_NAME" /bin/bash
