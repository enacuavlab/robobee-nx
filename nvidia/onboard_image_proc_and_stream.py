#!/usr/bin/env python3

#git clone https://github.com/ultralytics/yolov5
#cd yolov5 & pip install -r requirements.txt

import sys
sys.path.append('/home/pprz/.local/lib/python3.6/site-packages')
sys.path.append('/home/pprz/.local/lib/python3.6/site-packages/Pillow-9.2.0-py3.6-linux-aarch64.egg')
sys.path.append('/home/pprz/.local/lib/python3.6/site-packages/torchvision-0.11.0-py3.6-linux-aarch64.egg')

sys.path.append('/home/pprz/Projects/compagnon-software/pprzlink/lib/v2.0/python')
import pprzlink.udp
import pprzlink.message


import torch
import cv2
import numpy as np
import threading
import time
import socket

client_ip="127.0.0.1"
#client_ip="192.168.3.1"
post1_port=5700
post2_port=5800
downlink_port = 4250
pprz_port = 4246

WIDTH = 1280
HEIGHT = 720
FPS = 10

#model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/pprz/Projects/yolov5/runs/train/exp15/weights/best.pt')
model = torch.hub.load(r'/home/pprz/Projects/yolov5', 'custom', path=r'/home/pprz/Projects/yolov5/runs/train/exp15/weights/best.pt',source='local')

#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100) 

calib_file = '/home/pprz/Projects/imav2022/1280x720_calib_data/test.xml'
f = cv2.FileStorage(calib_file,cv2.FILE_STORAGE_READ)
cameraMatrix = f.getNode("matrix").mat()
distCoeffs = f.getNode("dist").mat()


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

param="()()"
param_ahrs="()"
param_gps="()"


def thread_cam_function(cond,cam):
  global frame
  while True:
    ret_val, frame = cam.read();
    if ret_val:
      with cond:
        cond.notify_all()


def thread_yolo_function(cond,lock,out):
  global frame,sock
  while True:
    with cond:
      cond.wait()
      img=frame.copy()
      results = model(img)
      if 0 in results.pandas().xyxy[0]['class']:
         img=(np.squeeze(results.render()))
         res=results.xyxy[0].cpu().detach().numpy()
         msgbuff="yolo "+param+" "+str(len(res))+" "
         for k in range(len(res)):
           msgbuff+="[%.2f %.2f %.2f %.2f %.2f]"%(res[k][0],res[k][1],res[k][2],res[k][3],res[k][4])
         msgbuff+="\n"
         with lock:
           sock.sendto(msgbuff.encode(), (client_ip, downlink_port))
#      out.write(img)


def thread_aruco_function(cond,lock,out):
  global frame,sock
  while True:
    with cond:
      cond.wait()
      gray3 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      corn, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray3, aruco_dict)
      if len(corn) == 0:
        gray1 = cv2.cvtColor(gray3, cv2.COLOR_GRAY2BGR)
      else:
        if (len(corn) > 0) and (len(ids) > 0):
          gray3b = cv2.aruco.drawDetectedMarkers(image=gray3, corners=corn, ids=ids, borderColor=(125, 125, 125))
          gray1 = cv2.cvtColor(gray3b, cv2.COLOR_GRAY2BGR)
          rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corn, 1, cameraMatrix, distCoeffs)
          msgbuff="aruco "+param+"["+str(len(ids))+"]"
          for k in range(len(rvecs)):
            msgbuff+="[%.2f %.2f %.2f][%.2f %.2f %.2f]"%(rvecs[k][0][0],rvecs[k][0][1],rvecs[k][0][2],tvecs[k][0][0],tvecs[k][0][1],tvecs[k][0][2])
          for k in range(len(corn)):
            msgbuff+="[%d %.2f %.2f %.2f %.2f]"%(ids[k],corn[k][0][0][0],corn[k][0][0][1],corn[k][0][1][0],corn[k][0][1][1])
          msgbuff+="\n"
        with lock:
           sock.sendto(msgbuff.encode(), (client_ip, downlink_port))
#      out.write(gray1)


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
  cam = cv2.VideoCapture("/dev/video2")
  if not cam.isOpened(): exit()

  gst_out_format="appsrc \
      ! video/x-raw,format=BGR ! queue ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=(string)I420 \
      ! nvv4l2h265enc insert-sps-pps=true bitrate=300000  ! h265parse ! rtph265pay pt=96 config-interval=1 ! udpsink host="
  out1 = cv2.VideoWriter(gst_out_format+client_ip+" port="+str(post1_port),cv2.CAP_GSTREAMER, 0, float(FPS), (int(WIDTH), int(HEIGHT)))
  out2 = cv2.VideoWriter(gst_out_format+client_ip+" port="+str(post2_port),cv2.CAP_GSTREAMER, 0, float(FPS), (int(WIDTH), int(HEIGHT)))
  if not out1.isOpened() and not out2.isOpened(): exit()

  thread_pprz = pprzlink.udp.UdpMessagesInterface(proccess_pprz_msg, False, 0, pprz_port, 'datalink')
  thread_pprz.start()

  lock = threading.Lock()
  cond = threading.Condition()
  thread_cam  = threading.Thread(target=thread_cam_function, args=(cond,cam,))
#  thread_yolo  = threading.Thread(target=thread_yolo_function, args=(cond,lock,out1,))
  thread_aruco  = threading.Thread(target=thread_aruco_function, args=(cond,lock,out2,))
  thread_cam.start()
#  thread_yolo.start()
  thread_aruco.start()

  while True :
    time.sleep(0.03)
    key = cv2.waitKey(1)
    if key == ord('q'): break
