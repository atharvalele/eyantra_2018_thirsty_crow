# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 18:22:08 2018

@author: Shirish
"""
import cv2
import numpy as np
import math
import cv2.aruco as ar

aruco_list=[1,2,3,4]
markerLength=100
#Code copied as it is from ArUco tutorial
img= cv2.imread('image_3.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
aruco_dict = ar.Dictionary_get(ar.DICT_5X5_250)
parameters= ar.DetectorParameters_create()
corners, ids, _ = ar.detectMarkers(gray, aruco_dict, parameters=parameters)
with np.load('C:\Hrishikesh\EYantra2018\Task 0\Task 0.2\Camera.npz') as X:
    camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
    rvec, tvec, obj= ar.estimatePoseSingleMarkers(corners,markerLength, camera_matrix, dist_coeff)
#To store image returned by the function
cv2.imwrite('Markerimg.jpg',ar.drawDetectedMarkers(img,corners,ids,(0,255,0)))
img1=cv2.imread('Markerimg.jpg')
cv2.imshow("Marker",img1)
cv2.waitKey()
#cv2.imshow("Im1",img1)
#To create an array of ArUco Id, centre, Rotation Vector and Translation vector
length=len(ids)
for j in range(0,length):
    aruco_list[j]=(ids[j],0,rvec[j],tvec[j])

#Code for calculating centre
x1= (corners[1][0][0][0]+corners[1][0][2][0])/2
print(x1)
x2= (corners[1][0][1][0]+corners[1][0][3][0])/2
print(x2)
ctrx= int((x1+x2)/2)
print(ctrx)
y1= (corners[1][0][0][1]+corners[1][0][3][1])/2
print(y1)
y2= (corners[1][0][1][1]+corners[1][0][2][1])/2
print(y2)
ctry= int((y1+y2)/2)
print(ctry)
for i in range(0,4):
    img=cv2.line(img,(corners[1][0][i][0],corners[1][0][i][1]),(ctrx,ctry),(0,0,255),4)
img=cv2.line(img,(corners[1][0][0][0],corners[1][0][0][1]),(corners[1][0][2][0],corners[1][0][2][1]),(255,0,255),4)
img=cv2.line(img,(corners[1][0][1][0],corners[1][0][1][1]),(corners[1][0][3][0],corners[1][0][3][1]),(255,0,255),4)
cv2.imshow("img2",img)
cv2.waitKey()
#cv2.imwrite('Line.jpg',cv2.line(img,corners[0],corners[1],(255,255,0),5))