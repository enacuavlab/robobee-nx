#!/usr/bin/env python3

# sudo apt install dkms
# sudo apt install v4l2loopback-dkms
#
# sudo modprobe v4l2loopback video_nr=2
# sudo modprobe -r v4l2loopback

WIDTH_OUT=1280
HEIGTH_OUT=720
WIDTH_IN=3264
HEIGTH_IN=2464
FPS=20
BITRATE=1000000


################################################################################
# NVIDIA

gst_out_format="appsrc \
      ! video/x-raw,format=BGR ! queue ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),width="+str(WIDTH_OUT)+",height="+str(HEIGTH_OUT)+",framerate="+str(FPS)+"/1,format=I420 \
      ! nvv4l2h265enc insert-sps-pps=true bitrate="+str(BITRATE)+" ! h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host=127.0.0.1 port=5700"


# Read capture device from onpencv
# capt_device="nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=20/1 ! nvvidconv flip-method=2 !  video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink"
# or
# Read capture device from V4l2loopback (same hight CPU consumption). This option, let to use the device for streaming or recording, even if opencv stop
'''
# stream to /dev/video2 for opencv
#
gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2


# add live stream
#
gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! tee name=t t. ! queue ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2 t. ! queue ! nvvidconv flip-method=2 ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=20/1,format=NV12" ! nvv4l2h265enc insert-sps-pps=true bitrate=1000000  ! h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host="127.0.0.1" port=5600 


# add raw image recording 
#
gst-launch-1.0 nvarguscamerasrc ! "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=20/1" ! tee name=t t. ! queue ! nvvidconv flip-method=2 ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=20/1,format=NV12" ! nvv4l2h265enc insert-sps-pps=true bitrate=1000000  ! h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host="127.0.0.1" port=5600 async=false t. ! queue ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! identity drop-allocation=1 ! v4l2sink device=/dev/video2 async=false t. ! queue ! videorate ! "video/x-raw(memory:NVMM),framerate=1/1"  ! nvv4l2h265enc ! h265parse ! matroskamux ! filesink location=/home/pprz/tmp3/test.mp4


'''
capt_device="/dev/video2"

# Read from client with
'''
gst-launch-1.0 udpsrc port=5600 ! application/x-rtp, encoding-name=H265, payload=96 ! rtph265depay ! h265parse ! queue ! avdec_h265 !  videoconvert ! autovideosink sync=false
gst-launch-1.0 udpsrc port=5700 ! application/x-rtp, encoding-name=H265, payload=96 ! rtph265depay ! h265parse ! queue ! avdec_h265 !  videoconvert ! autovideosink sync=false
'''

################################################################################
# PI
#
#gst_out_format="appsrc ! video/x-raw,format=BGR ! videoconvert ! video/x-raw,format=I420 ! v4l2h264enc ! video/x-h264,stream-format=byte-stream,alignment=au ! h264parse ! rtph264pay ! udpsink port=5600 host=192.168.3.1"

################################################################################
import cv2
import numpy as np;
import threading
import time


def thread_cam_function(cond,cam):
  global img
  while True:
    ret_val, img = cam.read();
    if ret_val:
      with cond:
        cond.notify_all()


def thread_blob_function(cond,lock,out):
  global img,sock
  hsv_low = (104,91,72)
  hsv_high = (179,255,110)
  while True:
    with cond:
      cond.wait()

      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, hsv_low, hsv_high)
      res = cv2.bitwise_and(img, img, mask=mask)
      img2 = img.copy()
      kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      contours = sorted(contours, key=cv2.contourArea, reverse=True)
      counter = 0
      for contour in contours[:3]:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(img2, (x, y), (x + w, y + h), (0, 255, 0), 3)
        print(counter,x,y,w,h)
        counter += 1
      out.write(img2)


if __name__ == '__main__':
  cam = cv2.VideoCapture(capt_device)
  if not cam.isOpened(): exit()

  out = cv2.VideoWriter(gst_out_format,cv2.CAP_GSTREAMER, 0, float(FPS), (int(WIDTH_IN), int(HEIGTH_IN)))
  if not out.isOpened(): exit()

  lock = threading.Lock()
  cond = threading.Condition()
  thread_cam  = threading.Thread(target=thread_cam_function, args=(cond,cam,))
  thread_blob  = threading.Thread(target=thread_blob_function, args=(cond,lock,out,))
  thread_cam.start()
  thread_blob.start()

  while True :
    time.sleep(0.03)
    key = cv2.waitKey(1)
    if key == ord('q'): break
