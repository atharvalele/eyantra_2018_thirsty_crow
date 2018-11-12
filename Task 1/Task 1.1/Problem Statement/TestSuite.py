import numpy as np
import cv2
import cv2.aruco as aruco
import math
import detect as d
"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: Thirsty Crow
*  MODULE: Task1.1
*  Filename: TestSuite.py
*  Version: 1.0.0  
*  Date: October 31, 2018
*  
*  Author: e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""
############### DO NOT MAKE ANY CHANGES IN THIS FILE #####################
results = []
"""
Function Name : check_ar_list_type()
Purpose: Checks if the format of the aruco_list is correct
"""

def check_ar_list_type(ar_list):
	ok_flag = True
	if len(ar_list):
		for k in ar_list:
			if type(k) is tuple and len(k) == 4:
				ok_flag = ok_flag*True
			else:
				ok_flag = ok_flag*False
			if type(k[1]) is tuple and len(k[1]) == 2:
				ok_flag = ok_flag*True
			else:
				ok_flag = ok_flag*False
			if type(k[2]) is np.ndarray and k[2].shape == (1,1,3):
				ok_flag = ok_flag*True
			else:
				ok_flag = ok_flag*False
			if type(k[3]) is np.ndarray and k[3].shape == (1,1,3):
				ok_flag = ok_flag*True
			else:
				ok_flag = ok_flag*False
	return ok_flag

"""
Function Name : draw_and_save()
Purpose: Draws axis, cube and cylinder on the image and saves them in different
         folders.
"""
def draw_and_save(img, ar_list, camera_matrix, dist_coeff):
	img_axis = np.zeros((480,640,3), dtype= np.uint8)
	img_cube = np.zeros((480,640,3), dtype= np.uint8)
	img_cylinder = np.zeros((480,640,3), dtype= np.uint8)
	img_axis[:,:,:] = img[:,:,:]; img_cube[:,:,:] = img[:,:,:]; img_cylinder[:,:,:] = img[:,:,:];
	for k in ar_list:
		
		d.drawAxis(img_axis, ar_list, k[0], camera_matrix, dist_coeff)
		cv2.imwrite("..\\SavedResults\\drawAxis\\axis" + str(imgnum) + ".jpg", img_axis)
		d.drawCube(img_cube, ar_list, k[0], camera_matrix, dist_coeff)
		cv2.imwrite("..\\SavedResults\\drawCube\\cube" + str(imgnum) + ".jpg", img_cube)
		d.drawCylinder(img_cylinder, ar_list, k[0], camera_matrix, dist_coeff)
		cv2.imwrite("..\\SavedResults\\drawCylinder\\cylinder" + str(imgnum) + ".jpg", img_cylinder)

"""
MAIN CODE
Purpose: Iterates through all the test cases and calls the function written by
         participants to draw images, cubes, cylinders on all the test case images.
         Saves 
"""
if __name__=="__main__":
	cam, dist = d.getCameraMatrix()
	for imgnum in range(1,9):
		image = cv2.imread("..\\TestCases\\image_" + str(imgnum) + ".jpg")
		ar = d.detect_markers(image, cam, dist)
		print (ar)
		print ("          ")
		ret = check_ar_list_type(ar)
		if ret == True:
			draw_and_save(image, ar, cam, dist)
		np_arr = np.array(ar, dtype=object)
		results.append(np_arr)
	np.savez("..\\SavedResults\\Results.npz", image1 = results[0], image2 = results[1], image3 = results[2], image4 = results[3], image5 = results[4], image6 = results[5], image7 = results[6], image8 = results[7])


	
