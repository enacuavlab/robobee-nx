#!/bin/bash

if [ $# != 5 ]; then exit; fi

WIDTH=$1
HEIGHT=$2
FPS=$3
IP=$4
PORT=$5

#IP="192.168.3.1"
#IP="127.0.0.1"
#PORT=5600

STORAGEPATH="/home/pprz/tmp2/calib_"$WIDTH"x"$HEIGHT
if [ -d $STORAGEPATH ]; then
  exit 0
fi

CAPS="video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT,framerate=$FPS/1,format=NV12"

mkdir $STORAGEPATH
gst-launch-1.0 nvarguscamerasrc \
  ! $CAPS \
  ! tee name=streams \
  ! queue \
  ! nvvidconv flip-method=0 \
  ! videorate \
  ! video/x-raw,framerate=1/1 \
  ! nvjpegenc \
  ! multifilesink post-messages=true location="$STORAGEPATH/frame%05d.jpg" streams. \
  ! queue \
  ! nvv4l2h265enc insert-sps-pps=true bitrate=2000000 \
  ! h265parse  \
  ! rtph265pay name=pay0 pt=96 config-interval=1 \
  ! udpsink host=$IP port=$PORT
