"""Library's"""
import cv2 as cv
import numpy as np

last_of_us111= "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/last_of_us.jpg"
last_of_us222= "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/last_of_us222.jpg"
last_of_us333= "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/last_of_us333.jpg"

xbox_kinect_sport = "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/xbox_kinect_sport.jpg"
xbox_kinect_sport222 = "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/xbox_kinect_sport222.jpg"
xbox_kinect_sport333 = "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/xbox_kinect_sport333.jpg"

lou = cv.imread(last_of_us111, 0)  #0 means in gray scale
xks = cv.imread(xbox_kinect_sport222, 0)
orb_algorithm = cv.ORB_create(nfeatures = 1000)

kp1, dec1 = orb_algorithm.detectAndCompute(lou, None)
kp2, dec2 = orb_algorithm.detectAndCompute(xks, None)

""" imgKp1 = cv.drawKeypoints(lou, kp1, None)
imgKp2 = cv.drawKeypoints(xks, kp2, None) """

bf = cv.BFMatcher()
matcher = bf.knnMatch(dec1, dec2, k=2)
good = []
for m, n in matcher:
        if m.distance < 0.75*n.distance:
                good.append([m])

print(len(good))
image3 = cv.drawMatchesKnn(lou, kp1, xks, kp2, good, None, flags = 2)

""" cv.imshow("lou",imgKp1)
cv.imshow("xks", imgKp2) """
cv.imshow("image 3", image3)
if cv.waitKey(0) & 0xFF == ord('q'):
        cv.destroyAllWindows()