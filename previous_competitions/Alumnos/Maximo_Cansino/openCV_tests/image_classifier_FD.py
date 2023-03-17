"""Library's"""
import cv2 as cv
import numpy as np
import os


#paths
image_folder = 'c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images'
last_of_us111= "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/last_of_us.jpg"
last_of_us222= "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/last_of_us222.jpg"
last_of_us333= "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/last_of_us333.jpg"
xbox_kinect_sport = "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/xbox_kinect_sport.jpg"
xbox_kinect_sport222 = "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/xbox_kinect_sport222.jpg"
xbox_kinect_sport333 = "c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/xbox_kinect_sport333.jpg"
video_path="c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/videos/lastOfUs.mp4"

#variables
images = []
classNames = []
folder_items = os.listdir(image_folder)
lou = cv.imread(last_of_us111, 0)  #0 means in gray scale
xks = cv.imread(xbox_kinect_sport222, 0)
orb_algorithm = cv.ORB_create(nfeatures = 1000)
cap = cv.VideoCapture(video_path)


for clss in folder_items:
    current_image = cv.imread(f"{image_folder}/{clss}", 0)
    images.append(current_image)
    classNames.append(os.path.splitext(clss)[0])

print(f"classes names -> {classNames}")
def findDescription(images):
    descriptionList=[]
    for img in images:
        kp, dec = orb_algorithm.detectAndCompute(img, None)
        descriptionList.append(dec)
        return descriptionList

descriptionList = findDescription(images)

def findID(img, descriptionListIDFinder, thres = 15):
    kp, dec = orb_algorithm.detectAndCompute(img, None)
    bf = cv.BFMatcher()
    matchList = []
    finalValue = -1
    try:
        for descriptor in descriptionListIDFinder:
            matcher = bf.knnMatch(descriptor, dec, k=2)
            good_features = []
            for m, n in matcher:
                if m.distance < 0.75*n.distance:
                    good.append([m])
            matchList.append(lend(good_features))
    except:
        pass
    if len(matchList) != 0:
        if max(matchList) > thres:
            finalValue = matchList.index(max(matchList))
    return finalValue

while True: 
    success, frame2 = cap.read()
    original_frame = frame2.copy()
    frame2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
    idd = findID(frame2, descriptionList)
    print (f"{idd}\t{classNames}")
    if idd != -1:
        cv.putText(original_frame, "something", (50,50), cv.FONT_HERSHEY_COMPLEX, 1, (0,69,255), 3)
    cv.imshow("video", original_frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
