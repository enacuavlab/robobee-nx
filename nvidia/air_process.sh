#!/bin/bash

#Client command to view stream
#/usr/bin/gst-launch-1.0  udpsrc port=5600 ! application/x-rtp, encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! xvimagesink 
#/usr/bin/gst-launch-1.0  udpsrc port=5700 ! application/x-rtp, encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! xvimagesink 
#/usr/bin/gst-launch-1.0  udpsrc port=5800 ! application/x-rtp, encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! xvimagesink 

if [ $# != 1 ]; then exit; fi
PIDFILE=$1

VIDEO_NBR=2
sudo modprobe v4l2loopback video_nr=$VIDEO_NBR
#sudo modprobe -r v4l2loopback

sleep 1

# MISSION
#########

#IP="192.168.3.1"
IP="127.0.0.1"
PORT=5600
DEV="/dev/video"$VIDEO_NBR
WIDTH=1280
HEIGHT=720
FPS=10
BITRATE=300000
CAPS="video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT,framerate=$FPS/1,format=NV12"

#gst-launch-1.0 nvarguscamerasrc ! $CAPS ! nvvidconv flip-method=2 ! $CAPS ! nvvidconv !  video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=I420 ! v4l2sink device=$DEV  2>&1 &
#echo $! >> $PIDFILE

#gst-launch-1.0 nvarguscamerasrc ! $CAPS ! nvvidconv flip-method=2 ! $CAPS ! nvv4l2h265enc insert-sps-pps=true bitrate=$BITRATE  ! h265parse ! rtph265pay pt=96 config-interval=1 ! udpsink host=$IP port=$PORT 
#echo $! >> $PIDFILE

#gst-launch-1.0 nvarguscamerasrc ! $CAPS ! nvvidconv flip-method=2 ! $CAPS ! tee name=t t. ! queue ! nvv4l2h265enc insert-sps-pps=true bitrate=$BITRATE  ! h265parse ! rtph265pay pt=96 config-interval=1 ! udpsink host=$IP port=$PORT t. ! queue ! nvvidconv !  video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=I420 ! v4l2sink device=$DEV 2>&1 &
#echo $! >> $PIDFILE

#sleep 1

#/usr/bin/python3 /home/pprz/Projects/imav2022/onboard_image_proc_and_stream.py 2>&1 &
#echo $! >> $PIDFILE

#/usr/bin/python3 /home/pprz/Projects/imav2022/onboard_proc.py $WIDTH $HEIGHT $DEV 2>&1 &
#echo $! >> $PIDFILE

# CALIBRATION 
##############

#WIDTH=1280
#HEIGHT=720
#FPS=10

#WIDTH=1640
#HEIGHT=1232
#FPS=10

WIDTH=1920
HEIGHT=1080
FPS=10

#WIDTH=3264
#HEIGHT=1848
#FPS=10

#WIDTH=3264
#HEIGHT=2464
#FPS=10

/home/pprz/Projects/imav2022/camera_calibration_capture.sh $WIDTH $HEIGHT $FPS $IP $PORT  2>&1 &
echo $! >> $PIDFILE
