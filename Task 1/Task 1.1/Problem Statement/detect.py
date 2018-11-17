import numpy as np
import cv2
import cv2.aruco as aruco
import math

"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: Thirsty Crow
*  MODULE: Task1.1
*  Filename: detect.py
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

####################### Define Utility Functions Here ##########################
"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
         distortion matrix saved in the calibration file.
"""


def getCameraMatrix():
    with np.load('System.npz') as X:
        camera_matrix, dist_coeff, _, _ = [X[i]
                                           for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    return camera_matrix, dist_coeff


"""
Function Name : sin()
Input: angle (in degrees)
Output: value of sine of angle specified
Purpose: Returns the sine of angle specified in degrees
"""


def sin(angle):
    return math.sin(math.radians(angle))


"""
Function Name : cos()
Input: angle (in degrees)
Output: value of cosine of angle specified
Purpose: Returns the cosine of angle specified in degrees
"""


def cos(angle):
    return math.cos(math.radians(angle))


################################################################################

"""
Function Name : detect_markers()
Input: img (numpy array), camera_matrix, dist_coeff
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
        centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
         distortion matrix as input and detects ArUco markers in the image. For each
         ArUco marker detected in image, paramters such as ID, centre coord, rvec
         and tvec are calculated and stored in a list in a prescribed format. The list
         is returned as output for the function
"""


def detect_markers(img, camera_matrix, dist_coeff):
    markerLength = 100
    aruco_list = []

    ######################## INSERT CODE HERE ########################
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict,
                                          parameters=parameters)

    with np.load("System.npz") as X:
        camera_matrix, dist_coeff, _, _ = [X[i]
                                           for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
        rvec, tvec, obj = aruco.estimatePoseSingleMarkers(
            corners, markerLength, camera_matrix, dist_coeff)

    for i in range(0, ids.size):
        # Calculation of center using average of the four corners
        x = (corners[i - 1][0][0][0] + corners[i - 1][0][1][0] + corners[i - 1][0][2][0] + corners[i - 1][0][3][0]) / 4
        y = (corners[i - 1][0][0][1] + corners[i - 1][0][1][1] + corners[i - 1][0][2][1] + corners[i - 1][0][3][1]) / 4
        center = (int(x), int(y))

        # Populating the aruco marker list
        marker = (ids[i][0], center, np.array([rvec[i]]), np.array([tvec[i]]))
        aruco_list.append(marker)
    ##################################################################

    return aruco_list


"""
Function Name : drawAxis()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws 3 mutually
         perpendicular axes on the specified aruco marker in the image and
         returns the modified image.
"""


def drawAxis(img, aruco_list, aruco_id, camera_matrix, dist_coeff):
    for x in aruco_list:
        if aruco_id == x[0]:
            rvec, tvec = x[2], x[3]
    markerLength = 100
    m = markerLength / 2
    pts = np.float32([[-m, m, 0], [m, m, 0], [-m, -m, 0], [-m, m, m]])
    pt_dict = {}
    imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
    for i in range(len(pts)):
        pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
    src = pt_dict[tuple(pts[0])]
    dst1 = pt_dict[tuple(pts[1])]
    dst2 = pt_dict[tuple(pts[2])]
    dst3 = pt_dict[tuple(pts[3])]

    img = cv2.line(img, src, dst1, (0, 255, 0), 4)
    img = cv2.line(img, src, dst2, (255, 0, 0), 4)
    img = cv2.line(img, src, dst3, (0, 0, 255), 4)
    return img


"""
Function Name : drawCube()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws a cube
         on the specified aruco marker in the image and returns the modified
         image.
"""


def drawCube(img, ar_list, ar_id, camera_matrix, dist_coeff):
    for x in ar_list:
        if ar_id == x[0]:
            rvec, tvec = x[2], x[3]
    markerLength = 100
    m = markerLength / 2
    ######################## INSERT CODE HERE ########################
    pts = np.float32([[-m, m, 0], [m, m, 0], [m, -m, 0], [-m, -m, 0], [-m, m, 2 * m], [m, m, 2 * m], [m, -m, 2 * m],
                      [-m, -m, 2 * m]])
    pt_dict = {}
    imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
    for i in range(len(pts)):
        pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())

    cubePoints = tuple([pt_dict[tuple(pts[i])] for i in range(0, 8)])

    for i in range(0, 5):
        # This will plot partial squares on the z axis and above it
        if i < 3:
            img = cv2.line(img, cubePoints[i], cubePoints[i + 1], (0, 0, 255), 4)
            img = cv2.line(img, cubePoints[i + 4], cubePoints[i + 5], (0, 0, 255), 4)
        # This finishes the squares
        if i == 0 or i == 4:
            img = cv2.line(img, cubePoints[i], cubePoints[i + 3], (0, 0, 255), 4)
        # This will connect the top and bottom squares
        if i != 4:
            img = cv2.line(img, cubePoints[i], cubePoints[i + 4], (0, 0, 255), 4)
    ##################################################################
    return img


"""
Function Name : drawCylinder()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws a cylinder
         on the specified aruco marker in the image and returns the modified
         image.
"""


def drawCylinder(img, ar_list, ar_id, camera_matrix, dist_coeff):
    for x in ar_list:
        if ar_id == x[0]:
            rvec, tvec = x[2], x[3]
    markerLength = 100
    radius = markerLength / 2
    height = markerLength * 1.5
    ######################## INSERT CODE HERE ########################
    # base points
    ptsBase = []
    for i in range(0, 24):
        x = radius * cos(i * 15)
        y = radius * sin(i * 15)
        ptsBase.append([x, y, 0])
    ptsBase = np.float32(ptsBase)
    pt_dict_base = {}
    imgptsBase, _ = cv2.projectPoints(ptsBase, rvec, tvec, camera_matrix, dist_coeff)
    for i in range(len(ptsBase)):
        pt_dict_base[tuple(ptsBase[i])] = tuple(imgptsBase[i].ravel())

    baseCirclePoints = tuple([pt_dict_base[tuple(ptsBase[i])] for i in range(0, len(ptsBase))])

    # top circle points
    ptsTop = []
    for i in range(0, 24):
        x = radius * cos(i * 15)
        y = radius * sin(i * 15)
        ptsTop.append([x, y, height])
    ptsTop = np.float32(ptsTop)
    pt_dict_top = {}
    imgptsTop, _ = cv2.projectPoints(ptsTop, rvec, tvec, camera_matrix, dist_coeff)
    for i in range(len(ptsTop)):
        pt_dict_top[tuple(ptsTop[i])] = tuple(imgptsTop[i].ravel())

    topCirclePoints = tuple([pt_dict_top[tuple(ptsTop[i])] for i in range(0, len(ptsTop))])

    # change the CirclePoints tuple to the format expected by drawContours()
    ctrBase = np.array(baseCirclePoints).reshape((-1, 1, 2)).astype(np.int32)
    ctrTop = np.array(topCirclePoints).reshape((-1, 1, 2)).astype(np.int32)

    cv2.drawContours(img, [ctrBase], -1, (255, 0, 0), 2)
    cv2.drawContours(img, [ctrTop], -1, (255, 0, 0), 2)

    # Drawing the lines
    pt_dict_ctr_base = {}
    pt_ctr_base = np.float32([[0, 0, 0]])
    imgpts_ctrBase, _ = cv2.projectPoints(pt_ctr_base, rvec, tvec, camera_matrix, dist_coeff)
    pt_dict_ctr_base[tuple(pt_ctr_base[0])] = tuple(imgpts_ctrBase[0].ravel())
    ctr_base = pt_dict_ctr_base[tuple(pt_ctr_base[0])]

    pt_dict_ctr_top = {}
    pt_ctr_top = np.float32([[0, 0, height]])
    imgpts_ctrTop, _ = cv2.projectPoints(pt_ctr_top, rvec, tvec, camera_matrix, dist_coeff)
    pt_dict_ctr_top[tuple(pt_ctr_top[0])] = tuple(imgpts_ctrTop[0].ravel())
    ctr_top = pt_dict_ctr_top[tuple(pt_ctr_top[0])]

    for i in range(0, 12):
        src = pt_dict_base[tuple(ptsBase[2 * i])]
        dst = pt_dict_top[tuple(ptsTop[2 * i])]
        img = cv2.line(img, src, dst, (255, 0, 0), 2)
        img = cv2.line(img, ctr_base, src, (255, 0, 0), 2)
        img = cv2.line(img, ctr_top, dst, (255, 0, 0), 2)
    ##################################################################
    return img


"""
MAIN CODE
This main code reads images from the test cases folder and converts them into
numpy array format using cv2.imread. Then it draws axis, cubes or cylinders on
the ArUco markers detected in the images.
"""

if __name__ == "__main__":
    cam, dist = getCameraMatrix()
    img = cv2.imread("../TestCases/image_1.jpg")
    aruco_list = detect_markers(img, cam, dist)
    for i in aruco_list:
        img = drawAxis(img, aruco_list, i[0], cam, dist)
        img = drawCube(img, aruco_list, i[0], cam, dist)
        img = drawCylinder(img, aruco_list, i[0], cam, dist)
    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
