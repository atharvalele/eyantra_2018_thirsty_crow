# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 18:22:08 2018

@author: Shirish
"""
import cv2
import numpy as np
import math
import cv2.aruco as aruco

def line(pt1,pt2):
    m= (pt2[1]-pt1[1])/(pt2[0]-pt1[0])      #Find slope
    c= m*pt1[0]-pt1[1]                      #Find intercept from c=mx-y
    return m,c
    
def intersect(m,c):
    D=m[0]-m[1]
    Dx= c[0]-c[1]
    Dy= -m[0]*c[1]+m[1]*c[0]
    x= Dx/D
    y= Dy/D
    return x,y
   
    
aruco_list=[1,2,3,4]
m=[[0,0],[0,0],[0,0]]    #Slope
c=[[0,0],[0,0],[0,0]]    #Intercept
x=[0,0,0]                #x coordinate
y=[0,0,0]                #y coordinate
markerLength=100
#Code copied as it is from ArUco tutorial
img= cv2.imread('image_8.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
parameters= aruco.DetectorParameters_create()
corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
with np.load('C:\Hrishikesh\EYantra2018\Task 0\Task 0.2\Camera.npz') as X:
    camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
    rvec, tvec, obj= aruco.estimatePoseSingleMarkers(corners,markerLength, camera_matrix, dist_coeff)
#To store image returned by the function
cv2.imwrite('Markerimg.jpg',ar.drawDetectedMarkers(img,corners,ids,(0,255,0)))
img1=cv2.imread('Markerimg.jpg')
cv2.imshow("Marker",img1)
cv2.waitKey()
#To create an array of ArUco Id, centre, Rotation Vector and Translation vector
length=len(ids)
for j in range(0,length):
    aruco_list[j]=(ids[j],0,rvec[j],tvec[j])
    for i in range(0,2):
        m1,c1=line(corners[j][0][i],corners[j][0][i+2])
        m[j][i]=m1
        c[j][i]=c1
    x1,y1=intersect(m[j],c[j])
    x[j]=int(x1)
    y[j]=int(y1)
    
#Plot a line from one corner to centre
    img=cv2.line(img,(x[j],y[j]),(corners[j][0][0][0],corners[j][0][0][1]),(255,0,255),1)
    #img=cv2.line(img,(577,325),(x[0],y[0]),(255,255,255),1)

cv2.imshow("img2",img)
cv2.waitKey()
#cv2.imwrite('Line.jpg',cv2.line(img,corners[0],corners[1],(255,255,0),5))