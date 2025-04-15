#!/bin/bash

gst-launch-1.0 -v v4l2src device=/dev/stereo_camera ! \
  video/x-raw,width=2560,height=720 ! videoconvert ! tee name=t \
  t. ! queue ! videocrop right=1280 ! videoconvert ! v4l2sink device=/dev/video17 \
  t. ! queue ! videocrop left=1280 ! videoconvert ! v4l2sink device=/dev/video18

#sudo modprobe v4l2loopback devices=3 video_nr=17,18,19 card_label="LeftCam","RightCam","StitchedCam"
