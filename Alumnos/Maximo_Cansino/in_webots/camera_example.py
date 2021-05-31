from controller import Robot
import cv2
import numpy as np


def getCameraSize(camera):
	print(f"Camera Width --> {camera.getWidth()}\t Camera Height--> {camera.getHeight()}")

def detectVisualSimple(image_data, camera):

	coords_list = []
	img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
	img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])


	#convert from BGR to HSV color space
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#apply threshold
	thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]

	# draw all contours in green and accepted ones in red
	contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	for c in contours:
		if cv2.contourArea(c) > 1000:
			coords = list(c[0][0])
			coords_list.append(coords)
			print("Victim at x="+str(coords[0])+" y="+str(coords[1]))

	return coords_list


robot = Robot()
timeStep = 32

camera1 = robot.getDevice("camera1")
#camera2 = robot.getDevice("camera2")
camera1.enable(timeStep)
#camera2.enable(timeStep)
while robot.step(timeStep) != -1:
	camera1Visual = camera1.getImage()
	#camera2Visual = camera2.getImage()
	camera1Visual = np.frombuffer(camera1Visual, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
	camera1Visual = np.array(camera1Visual ,dtype=np.uint8)
	#panel =cv2.hconcat(camera1Visual, camera2Visual)
	getCameraSize(camera1)
	cv2.imshow("panel", camera1Visual)

	
	
