#!/usr/bin/env python3

#git clone https://github.com/ultralytics/yolov5
#cd yolov5 & pip install -r requirements.txt

import sys
sys.path.append('/home/pprz/.local/lib/python3.6/site-packages')
sys.path.append('/home/pprz/.local/lib/python3.6/site-packages/Pillow-9.2.0-py3.6-linux-aarch64.egg')
sys.path.append('/home/pprz/.local/lib/python3.6/site-packages/torchvision-0.11.0-py3.6-linux-aarch64.egg')

sys.path.append('/home/pprz/Projects/compagnon-software/pprzlink/lib/v2.0/python')
from pprzlink.serial import SerialMessagesInterface
from pprzlink.message import PprzMessage
import cv2
import numpy as np
import threading
import time
import socket

#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100) 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250) 

MARKER_SIZE=200

SERIALDEV='/dev/ttyTHS1'

CLIENT_IP="127.0.0.1"
CLIENT_PORT=4250
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def camera_calibration(calib_file):
  f = cv2.FileStorage(calib_file,cv2.FILE_STORAGE_READ)
  cameraMatrix = f.getNode("matrix").mat()
  distCoeffs = f.getNode("dist").mat()
  return cameraMatrix,distCoeffs;


def thread_cam_function(cond,cam):
  global frame
  while True:
    ret_val, frame = cam.read();
    if ret_val:
      with cond:
        cond.notify_all()


def dummy(*arg):
  pass

def thread_aruco_function(cond,thread_pprz,matrix,coeff):
  global frame
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
          rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corn, MARKER_SIZE, matrix, coeff) # 200 aruco size
          for k in range(len(rvecs)):
            msg = PprzMessage('datalink', 'TARGET_POS')
            msg['ac_id'] = 42
            msg['target_id'] = ids[k]
            msg['lat']= tvecs[k][0][0]
            msg['lon']= tvecs[k][0][1]
            msg['alt']= tvecs[k][0][2]
            msg['speed']= 0
            msg['climb']= 0
            msg['course']= 0
            thread_pprz.send(msg, 0)
            msgbuff="[TARGET_POS %d %d %.2f %.2f %.2f %d %d %d]\n"\
              %(msg['ac_id'],msg['target_id'],msg['lat'],msg['lon'], msg['alt'] ,msg['speed'],msg['climb'],msg['course'])
            sock.sendto(msgbuff.encode(), (CLIENT_IP, CLIENT_PORT))
            print(msgbuff)
            #print("TARGET_POS %d %d %.2f %.2f %.2f %d %d %d"
            #  %(msg['ac_id'],msg['target_id'],msg['lat'],msg['lon'], msg['alt'] ,msg['speed'],msg['climb'],msg['course']))


if __name__ == '__main__':
  if len(sys.argv)!=4:
    exit()
  width=sys.argv[1:2][0]
  height=sys.argv[2:3][0]
  videodev=sys.argv[3:4][0]
  cam = cv2.VideoCapture(videodev)
  if not cam.isOpened(): exit()

  thread_pprz = SerialMessagesInterface(dummy, False, SERIALDEV)
  thread_pprz.start()

  cond = threading.Condition()
  thread_cam  = threading.Thread(target=thread_cam_function, args=(cond,cam,))
  thread_cam.start()

  matrix,coeff = camera_calibration("/home/pprz/Projects/imav2022/calibration/calib_"+width+"x"+height+"/calibration.xml")
#  matrix,coeff = camera_calibration("/home/pprz/tmp2/calib_"+width+"x"+height+"/calibration.xml")
  thread_aruco = threading.Thread(target=thread_aruco_function, args=(cond,thread_pprz,matrix,coeff,))
  thread_aruco.start()

  while True :
    time.sleep(0.03)
    key = cv2.waitKey(1)
    if key == ord('q'): break
