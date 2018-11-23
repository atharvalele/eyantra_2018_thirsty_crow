import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image

texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None
cap = cv2.VideoCapture(0)
INVERSE_MATRIX = np.array([[1.0, 1.0, 1.0, 1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [1.0, 1.0, 1.0, 1.0]])


def getCameraMatrix():
    global camera_matrix, dist_coeff
    with np.load("..\\..\\Camera.npz") as X:
        camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


def main():
    glutInit()
    getCameraMatrix()
    glutInitWindowSize(640, 480)
    glutInitWindowPosition(625, 100)
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
    window_id = glutCreateWindow("OpenGL")
    init_gl()
    glutDisplayFunc(drawGLScene)
    glutIdleFunc(drawGLScene)
    glutReshapeFunc(resize)
    glutMainLoop()


def init_gl():
    global texture_object, texture_background
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    texture_background = glGenTextures(1)
    texture_object = glGenTextures(1)


def resize(w, h):
    ratio = 1.0 * w / h
    glMatrixMode(GL_PROJECTION)
    glViewport(0, 0, w, h)
    gluPerspective(45, ratio, 0.1, 100.0)


def detect_markers(img):
    aruco_list = []
    ################################################################
    #################### Same code as Task 1.1 #####################
    markerLength = 100
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict,
                                          parameters=parameters)

    with np.load("..\\..\\Camera.npz") as X:
        camera_matrix, dist_coeff, _, _ = [X[i]
                                           for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
        rvec, tvec, obj = aruco.estimatePoseSingleMarkers(
            corners, markerLength, camera_matrix, dist_coeff)

    if (type(ids) == None):
        return aruco_list
    try:
        for i in range(0, ids.size):
            # Calculation of center using average of the four corners
            x = (corners[i - 1][0][0][0] + corners[i - 1][0][1][0] + corners[i - 1][0][2][0] + corners[i - 1][0][3][0]) / 4
            y = (corners[i - 1][0][0][1] + corners[i - 1][0][1][1] + corners[i - 1][0][2][1] + corners[i - 1][0][3][1]) / 4
            center = (int(x), int(y))

            # Populating the aruco marker list
            marker = (ids[i][0], center, np.array([rvec[i]]), np.array([tvec[i]]))
            aruco_list.append(marker)
    except:
        print("No aruco markers in view")


    ################################################################
    return aruco_list

def drawGLScene():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    ar_list = []
    ret, frame = cap.read()
    if ret == True:
        draw_background(frame)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        ar_list = detect_markers(frame)
        for i in ar_list:
            if i[0] == 8:
                overlay(frame, ar_list, i[0], "texture_1.png")
            if i[0] == 2:
                overlay(frame, ar_list, i[0], "texture_2.png")
            if i[0] == 7:
                overlay(frame, ar_list, i[0], "texture_3.png")
            if i[0] == 6:
                overlay(frame, ar_list, i[0], "texture_4.png")

    cv2.imshow('frame', frame)
    cv2.waitKey(1)

    glutSwapBuffers()

def draw_background(image):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    # convert image to OpenGL texture format
    bg_image = cv2.flip(image, 1)
    bg_image = Image.fromarray(bg_image)
    ix = bg_image.size[0]
    iy = bg_image.size[1]
    bg_image = bg_image.tobytes("raw", "BGRX", 0, -1)
    glEnable(GL_TEXTURE_2D)
    # create background texture
    glBindTexture(GL_TEXTURE_2D, texture_background)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
    # draw background
    glPushMatrix()
    glRotatef(0, 0, 0, 1)
    glBegin(GL_QUADS)
    glTexCoord2d(0.0, 0.0)
    glVertex2d(-1.0, -1.0)
    glTexCoord2d(1.0, 0.0)
    glVertex2d(+1.0, -1.0)
    glTexCoord2d(1.0, 1.0)
    glVertex2d(+1.0, +1.0)
    glTexCoord2d(0.0, 1.0)
    glVertex2d(-1.0, +1.0)
    glEnd()
    # draw_background(bg_image)
    glPopMatrix()
    glTranslatef(0.0, 0.0, -1.0)

def init_object_texture(image_filepath):
    glEnable(GL_TEXTURE_2D)
    tex_image = cv2.imread(image_filepath)
    tex_image = Image.fromarray(tex_image)
    ix = tex_image.size[0]
    iy = tex_image.size[1]
    tex_image = tex_image.tobytes("raw", "BGRX", 0, -1)
    # create background texture
    glBindTexture(GL_TEXTURE_2D, texture_object)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_image)
    # glPushMatrix()
    # glRotatef(0, 0, 0, 1)
    # glBegin(GL_QUADS)
    # glTexCoord2d(0.0, 0.0)
    # glVertex2d(-1.0, -1.0)
    # glTexCoord2d(1.0, 0.0)
    # glVertex2d(+1.0, -1.0)
    # glTexCoord2d(1.0, 1.0)
    # glVertex2d(+1.0, +1.0)
    # glTexCoord2d(0.0, 1.0)
    # glVertex2d(-1.0, +1.0)
    # glEnd()
    # # draw_background(bg_image)
    # glPopMatrix()
    # glTranslatef(0.0, 0.0, -1.0)
    return None

def overlay(img, ar_list, ar_id, texture_file):
    for x in ar_list:
        if ar_id == x[0]:
            centre, rvec, tvecs = x[1], x[2], x[3]
    rmtx = cv2.Rodrigues(rvec)[0]
    view_matrix = np.array([[rmtx[0][0], rmtx[0][1], rmtx[0][2], 0],
                            [rmtx[1][0], rmtx[1][1], rmtx[1][2], 0],
                            [rmtx[2][0], rmtx[2][1], rmtx[2][2], 4],
                            [0.0, 0.0, 0.0, 1.0]])
    view_matrix = view_matrix * INVERSE_MATRIX
    view_matrix = np.transpose(view_matrix)

    init_object_texture(texture_file)
    glPushMatrix()
    glLoadMatrixd(view_matrix)
    glutSolidTeapot(0.5)
    glPopMatrix()

if __name__ == "__main__":
    main()
