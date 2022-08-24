#!/usr/bin/env python3

#git clone https://github.com/ultralytics/yolov5
#cd yolov5 & pip install -r requirements.txt

import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2
import pdb
import sys
import time
import threading 

#client_ip="127.0.0.1"
client_ip="192.168.3.1"
pre_port=5600
post1_port=5700
post2_port=5800

#model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/pprz/Projects/yolov5/runs/train/exp15/weights/best.pt')
model = torch.hub.load(r'/home/pprz/Projects/yolov5', 'custom', path=r'/home/pprz/Projects/yolov5/runs/train/exp15/weights/best.pt',source='local')

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()


def thread_yolo_function(cond,out):
  global frame
  while True:
    with cond:
      cond.wait()
      image = frame.copy()
      results = model(image)
      print(results.pred[0])
      print('XYXY : ',results.xyxy)
      out.write(np.squeeze(results.render()))


def thread_aruco_function(cond,out):
  global frame
  while True:
    with cond:
      cond.wait()
      image = frame.copy()
      gray3 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray3, aruco_dict)
      if len(corners) == 0:
        gray1 = cv2.cvtColor(gray3, cv2.COLOR_GRAY2BGR)
      else:
        gray3b = cv2.aruco.drawDetectedMarkers(image=gray3, corners=corners, ids=ids, borderColor=(0, 255, 0))
        gray1 = cv2.cvtColor(gray3b, cv2.COLOR_GRAY2BGR)
  #      rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 1, cameraMatrix, distCoeffs)
  #      for rvec, tvec in zip(rvecs, tvecs):
  #        cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 1)
      out.write(gray1)


def thread_cam_function(cond,cam):
  global frame
  while True:
    ret_val, image = cam.read();
    if ret_val:
      with cond:
        frame=cv2.cvtColor(image,cv2.COLOR_YUV2BGR_I420)
        print("PUT")
        cond.notifyAll()


if __name__ == '__main__':
  cam = cv2.VideoCapture("nvarguscamerasrc \
      ! video/x-raw(memory:NVMM),width=(int)1280,height=(int)720,format=(string)NV12,framerate=(fraction)10/1 \
      ! tee name=t t. ! queue ! nvv4l2h264enc ! h264parse ! rtph264pay pt=96 config-interval=1 ! udpsink host="+client_ip+" port="+str(pre_port)+" t. \
      ! queue ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! appsink") 
  if not cam.isOpened(): exit()

  w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
  h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
  fps = cam.get(cv2.CAP_PROP_FPS)
  gst_out_format="appsrc \
      ! video/x-raw,format=BGR ! queue ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=(string)I420 \
      ! nvv4l2h264enc ! h264parse ! rtph264pay pt=96 config-interval=1 ! udpsink host="
  out1 = cv2.VideoWriter(gst_out_format+client_ip+" port="+str(post1_port),cv2.CAP_GSTREAMER, 0, float(fps), (int(w), int(h)))
  out2 = cv2.VideoWriter(gst_out_format+client_ip+" port="+str(post2_port),cv2.CAP_GSTREAMER, 0, float(fps), (int(w), int(h)))
  if not out1.isOpened() or not out2.isOpened(): exit()

  cond = threading.Condition()
  thread_cam  = threading.Thread(target=thread_cam_function, args=(cond,cam,))
  thread_aruco  = threading.Thread(target=thread_aruco_function, args=(cond,out1,))
  thread_yolo  = threading.Thread(target=thread_yolo_function, args=(cond,out2,))
  thread_cam.start()
  thread_yolo.start()
  thread_aruco.start()

  while True :
    time.sleep(0.03)
    key = cv2.waitKey(1)
    if key == ord('q'): break
