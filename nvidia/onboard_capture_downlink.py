#!/usr/bin/env python3

import os
import sys
import threading
import time
import socket


sys.path.append('/home/pprz/Projects/compagnon-software/pprzlink/lib/v2.0/python')
import pprzlink.udp
import pprzlink.message

LOCAL_DOWNLINK_PORT = 4246
INTER_APP_PORT = 4250

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def proccess_downlink_msg(sender,address,msg,length,receiver_id=None, component_id=None):
  if msg.name=='AHRS_QUAT_INT': 
    d={}
    for idx, f in enumerate(msg.fieldnames): d[f]=msg.fieldvalues[idx]
    msgbuff='%f %d %d %d %d'%(d['weight'],d['imu_qi'],d[ 'imu_qx'],d['imu_qy'],d['imu_qz'])
    sock.sendto(msgbuff.encode(), ('127.0.0.1', INTER_APP_PORT))


if __name__ == '__main__':

  interface=pprzlink.udp.UdpMessagesInterface(proccess_downlink_msg, False, 0, LOCAL_DOWNLINK_PORT, 'datalink')

  try:
    interface.start()
    time.sleep(0.1)
    while interface.isAlive():
      interface.join(1)

  except (KeyboardInterrupt, SystemExit):
    print('Shutting down...')
    interface.stop()
    exit()
