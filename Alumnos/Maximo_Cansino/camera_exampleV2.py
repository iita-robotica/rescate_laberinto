from controller import Robot, Camera
import cv2 # Include OpenCV library
import numpy as np # For python, include numpy as well

robot = Robot()
camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
timestep = int(robot.getBasicTimeStep())

camera1.enable(timestep)
camera2.enable(timestep)
while robot.step(timestep) != -1:
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))

    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))

    camera_panel = cv2.hconcat((image2,image1))
    cv2.imshow("camera_panel", camera_panel)
    cv2.waitKey(1) 