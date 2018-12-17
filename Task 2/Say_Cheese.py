import cv2
import numpy
import cv2.aruco as aruco

def draw_aruco(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    img = aruco.drawDetectedMarkers(img, corners, ids)
    return img

cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if ret == True:
        frame = draw_aruco(frame)        
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.waitKey(1) & 0xFF == ord('k'):
            cv2.imwrite("arena_image.png", frame)
cap.release()
cv2.destroyAllWindows()
