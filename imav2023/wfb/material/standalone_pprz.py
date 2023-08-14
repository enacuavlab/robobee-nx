#!/usr/bin/env python3

import sys
#sys.path.append('/home/pprz/Projects/tobedeleted/compagnon-software/pprzlink/lib/v2.0/python')
sys.path.append('/home/pprz/Projects/pprzlink/lib/v2.0/python')
import pprzlink.udp
import pprzlink.message


import cv2
import numpy as np
import threading
import time
import socket

pprz_port = 4244

param="()()"
param_ahrs="()"
param_gps="()"

def proccess_pprz_msg(sender,address,msg,length,receiver_id=None, component_id=None):
  global param,param_ahrs,param_gps
  param_updated=False
  if msg.name=='AHRS_QUAT_INT':
    d={}
    for idx, f in enumerate(msg.fieldnames): d[f]=msg.fieldvalues[idx]
    param_ahrs='(%.2f %.2f %.2f %.2f %.2f)'%(d['weight'],d['body_qi'],d[ 'body_qx'],d['body_qy'],d['body_qz'])
    print(param_ahrs)
    param_updated=True
  if msg.name=='ROTORCRAFT_FP':
    d={}
    for idx, f in enumerate(msg.fieldnames): d[f]=msg.fieldvalues[idx]
    param_gps='(%.2f %.2f %.2f)'%(d['east'],d['north'],d[ 'up'])
    print(param_gps)
    param_updated=True
  if param_updated:
    param=param_gps+param_ahrs
    param_updated=False

if __name__ == '__main__':
  thread_pprz = pprzlink.udp.UdpMessagesInterface(proccess_pprz_msg, False, 0, pprz_port, 'datalink')
  thread_pprz.start()

  while True :
    time.sleep(0.03)
    key = cv2.waitKey(1)
    if key == ord('q'): break
