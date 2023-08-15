#!/usr/bin/env python3

'''
Usage
./blob_calibration.py dummy/IMG_1380.jpg 
./blob_calibration.py dummy/IMG_1380.jpg 104 179 91 255 72 110
'''
import cv2
import sys
import numpy as np


window_name = 'frame'
cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
tracks = [('H_low',(0,179,0)),('H_high',(179,179,179)),('S_low',(0,255,0)),
          ('S_high',(255,255,255)),('V_low',(0,255,0)),(',V_high',(255,255,255))] 


def callback(x):
  global window_name,img,hsv

  for i,tr in enumerate(tracks):
    tmpA = list(tracks[i])
    tmpB = list(tmpA[1])
    tmpB[2] = cv2.getTrackbarPos(tr[0],window_name)
    tmpA[1] = tuple(tmpB)
    tracks[i] = tuple(tmpA)

  hsv_low  = np.array([tracks[0][1][2],tracks[2][1][2],tracks[4][1][2]],np.uint8)
  hsv_high = np.array([tracks[1][1][2],tracks[3][1][2],tracks[5][1][2]],np.uint8)

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
  cv2.imshow(window_name,np.concatenate((img,res,img2),axis=1))


def main(argv):
  global img,hsv
  img = cv2.imread(sys.argv[1])
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

  for tr in tracks: cv2.createTrackbar(tr[0],window_name,tr[1][0],tr[1][1],callback)
  if (len(sys.argv)>2):
    for i,tr in enumerate(tracks):
      tmpA = list(tracks[i])
      tmpB = list(tmpA[1])
      tmpB[2] = cv2.setTrackbarPos(tr[0],window_name,int(sys.argv[i+2]))
      tmpA[1] = tuple(tmpB)
      tracks[i] = tuple(tmpA)

  while(True): # just work on callback
    cv2.waitKey(1) & 0xFF  # wait for the windows cross or ctrl+c to exit
    if not cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE): break
  print("---------------------------------")
  print("hsv low and high")
  print(tracks[0][1][2],tracks[1][1][2],tracks[2][1][2],tracks[3][1][2],tracks[4][1][2],tracks[5][1][2])
  print("---------------------------------")


if __name__ == "__main__":
  if (len(sys.argv)>1) :  main(sys.argv)
