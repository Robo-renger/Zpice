#!/bin/bash

# Check if a device name is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <device_name>"
  exit 1
fi

DEVICE=$1

gst-launch-1.0 v4l2src device=$DEVICE ! video/x-raw,width=2560,height=720 \
  ! videoconvert \
  ! tee name=t \
  t. ! queue ! videoscale ! video/x-raw,width=1280,height=720 ! videobox left=0 right=-1280 ! v4l2sink device=/dev/stereo_left \
  t. ! queue ! videoscale ! video/x-raw,width=1280,height=720 ! videobox left=-1280 right=0 ! v4l2sink device=/dev/stereo_right