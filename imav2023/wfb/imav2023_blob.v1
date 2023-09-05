#!/usr/bin/env python3

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

capt_device="/dev/video2"

################################################################################
# PI
#
################################################################################
import sys
sys.path.append('/home/pprz/Projects/pprzlink/lib/v2.0/python')
import pprzlink.udp
import pprzlink.message

import cv2
import numpy as np;
import threading
import time
import socket
LOCAL_IP="127.0.0.1"
PPRZ_PORT = 4244
WFB_PORT = 5900

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
param="()()"
param_ahrs="()"
param_gps="()"


def thread_cam_function(cond,cam):
  global img
  while True:
    ret_val, img = cam.read();
    if ret_val:
      with cond:
        cond.notify_all()


def thread_blob_function(cond,lock,out):
  global img,sock
  hsv_low = (107,0,0)
  hsv_high = (179,255,255)
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
      msg=""
      for contour in contours[:3]:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(img2, (x, y), (x + w, y + h), (0, 255, 0), 3)
        msg += "    ["+str(counter)+param+str(x)+str(y)+str(w)+str(h)+"]"
        counter += 1
      out.write(img2)
      if msg:
        print(msg)
        msg+="\n"
        sock.sendto(msg.encode(), (LOCAL_IP, WFB_PORT))


def proccess_pprz_msg(sender,address,msg,length,receiver_id=None, component_id=None):
  global param,param_ahrs,param_gps
  param_updated=False
  if msg.name=='AHRS_QUAT_INT':
    d={}
    for idx, f in enumerate(msg.fieldnames): d[f]=msg.fieldvalues[idx]
    param_ahrs='(%.2f %.2f %.2f %.2f %.2f)'%(d['weight'],d['body_qi'],d[ 'body_qx'],d['body_qy'],d['body_qz'])
    param_updated=True
  if msg.name=='ROTORCRAFT_FP':
    d={}
    for idx, f in enumerate(msg.fieldnames): d[f]=msg.fieldvalues[idx]
    param_gps='(%.2f %.2f %.2f)'%(d['east'],d['north'],d[ 'up'])
    param_updated=True
  if param_updated:
    param=param_gps+param_ahrs
    param_updated=False


if __name__ == '__main__':
  cam = cv2.VideoCapture(capt_device)
  if not cam.isOpened(): exit()

  out = cv2.VideoWriter(gst_out_format,cv2.CAP_GSTREAMER, 0, float(FPS), (int(WIDTH_IN), int(HEIGTH_IN)))
  if not out.isOpened(): exit()

  lock = threading.Lock()
  cond = threading.Condition()
  thread_cam  = threading.Thread(target=thread_cam_function, args=(cond,cam,))
  thread_blob  = threading.Thread(target=thread_blob_function, args=(cond,lock,out,))
  thread_pprz = pprzlink.udp.UdpMessagesInterface(proccess_pprz_msg, False, 0, PPRZ_PORT, 'datalink')

  thread_pprz.start()
  thread_cam.start()
  thread_blob.start()

  while True :
    time.sleep(0.03)
    key = cv2.waitKey(1)
    if key == ord('q'): break
