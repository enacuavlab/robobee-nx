#!/bin/bash

modprobe v4l2loopback video_nr=2
# sudo modprobe -r v4l2loopback


if [ $# != 1 ]; then exit; fi
PIDFILE=$1

################################################################################
# NVIDIA CAMERA

# stream to /dev/video2 for opencv
#
#gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2 2>&1 &
#echo $! >> $PIDFILE

# add live stream
#
#gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! tee name=t t. ! queue ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2 t. ! queue ! nvvidconv flip-method=2 ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=20/1,format=NV12" ! nvv4l2h265enc insert-sps-pps=true bitrate=1000000  ! h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host="127.0.0.1" port=5600 2>&1 &
#echo $! >> $PIDFILE

# Read from client with
#gst-launch-1.0 udpsrc port=5600 ! application/x-rtp, encoding-name=H265, payload=96 ! rtph265depay ! h265parse ! queue ! avdec_h265 !  videoconvert ! autovideosink sync=false
#gst-launch-1.0 udpsrc port=5700 ! application/x-rtp, encoding-name=H265, payload=96 ! rtph265depay ! h265parse ! queue ! avdec_h265 !  videoconvert ! autovideosink sync=false

################################################################################
# OPENCV IMAGE PROCESING 

#/usr/bin/python3 /home/pprz/Projects/imav2023/imav2023_blob.py 2>&1 &
#echo $! >> $PIDFILE


################################################################################
# TO BE TESTED
################################################################################
# RECORD if file does not exist and space is available. 
# Monitor free space, and launch backup stream on limit.

if [[ ! -f "/home/pprz/tmp3/test.mp4" ]]; then
  `gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! tee name=t t. ! queue ! nvvidconv flip-method=2 ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=20/1,format=NV12" ! nvv4l2h265enc insert-sps-pps=true bitrate=1000000  ! h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host="127.0.0.1" port=5600 async=false t. ! queue ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2 async=false t. ! queue ! videorate ! "video/x-raw(memory:NVMM),framerate=1/1"  ! nvv4l2h265enc ! h265parse ! matroskamux ! filesink location=/home/pprz/tmp3/test.mp4 2>&1 & echo $! > /tmp/pid` & 
  cat /tmp/pid  >> $PIDFILE
  while true; do
    left=`df $PWD | awk 'NR==2{print $4}'`
    if [[ $left -lt  5000000 ]];  then break; fi
    sleep 1
  done
  kill -9 `cat /tmp/pid`
fi

`gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! tee name=t t. ! queue ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2 t. ! queue ! nvvidconv flip-method=2 ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=20/1,format=NV12" ! nvv4l2h265enc insert-sps-pps=true bitrate=1000000  ! h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host="127.0.0.1" port=5600  2>&1 & echo $! > /tmp/pid` & 
cat /tmp/pid  >> $PIDFILE
