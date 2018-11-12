
print ("==============================================")
result = []
try:
	import platform
	sysver = platform.uname()
	isysver = 1;
	result.append(sysver)
	# print "System version is:", sysver

except ImportError:
	# print "No Information about OS"
	sysver = ""
	result.append(sysver)
	isysver = 0;
	pass
	#sys.exit(0)

try: 
	import platform

	pyver = platform.python_version()
	print ("Required Python Version is 3.5.4")
	if (pyver[0:5] == '3.5.4'):
		print ("Installed Python Version is: ", pyver)
		print ("Python Installation is OK!!")
		print ("==============================================")
	else:
		print ("Installed Python Version is: ", pyver)
		print ("Python Installation is NOT OK .. Please re-install the correct version!!")
		print ("==============================================")
except ImportError:
	print ("Import Error - re-check installation procedure ")
	pyver = ""
	result.append(pyver)
	pass

try:
	import numpy
	numpyver = numpy.__version__
	print ("Required Numpy version is: 1.14.x")
	if (numpyver[0:4] == '1.14'):
		print ("Installed Numpy Version is: ", numpyver)
		print ("Numpy Installation is OK!!")
		print ("==============================================")
	else:
		print ("Installed Numpy Version is: ", numpyver)
		print ("Numpy Installation is NOT OK .. Please re-install the correct version!!")
		print ("==============================================")
except ImportError:
	print ("Import Error - re-check installation procedure.")
	pass

try:
	import cv2
	#import cv2.aruco
	cv2ver = cv2.__version__
	correct_cv_version = None
	print ("Required OpenCV version is: 3.4.3")
	if (cv2ver == '3.4.3'):
		print ("Installed OpenCV Version is: ", cv2ver)
		correct_cv_version = 1
	else:
		print ("Installed OpenCV Version is: ", cv2ver)
		correct_cv_version = 0
except ImportError:
	print ("Import Error - re-check installation procedure.")
	pass
if (correct_cv_version == 1):
    try:
        import cv2.aruco
        print ("OpenCV Aruco Library correctly installed")
        print ("OpenCV Installation is OK!!")
        print ("==============================================")
    except ImportError:
        print ("OpenCV Aruco Library not correctly installed. Re-check installation procedure. ")
        pass
elif (correct_cv_version == 0):
    print ("OpenCV Installation is NOT OK .. Please re-install the correct version!!")
    print ("==============================================")

else:
    pass

try:
    import pygame
    pygamever = pygame.__version__
    print ("Required Pygame version is: 1.9.4")
    if (pygamever == '1.9.4'):
        print ("Installed Pygame Version is: ", pygamever)
        print ("Pygame Installation is OK!!")
        print ("==============================================")
    else:
        print ("Installed Pygame Version is: ", pygamever)
        print ("Pygame Installation is NOT OK .. Please re-install the correct version!!")
        print ("==============================================")
except ImportError:
    print ("Import Error - re-check installation procedure.")
    pass

try:
    from PIL import Image
    Imagever = Image.__version__
    print ("Required PIL version is: 5.2.0")
    if (Imagever == '5.2.0'):
        print ("Installed PIL Version is: ", Imagever)
        print ("PIL Installation is OK!!")
        print ("==============================================")
    else:
        print ("Installed PIL Version is: ", Imagever)
        print ("PIL Installation is NOT OK .. Please re-install the correct version!!")
        print ("==============================================")
except ImportError:
    print ("Import Error - re-check installation procedure.")
    pass

try:
    import OpenGL.GL
    import OpenGL.GLU
    import OpenGL.GLUT
    GLver = OpenGL.__version__
    print ("Required OpenGL version is: 3.1.2")
    if (GLver == '3.1.2'):
        print ("Installed OpenGL Version is: ", GLver)
        print ("OpenGL Installation is OK!!")
        print ("==============================================")
    else:
        print ("Installed OpenGL Version is: ", GLver)
        print ("OpenGL Installation is NOT OK .. Please re-install the correct version!!")
        print ("==============================================")
except ImportError:
    print ("Import Error - re-check installation procedure.")
    pass


			
