import cv2 as cv
import numpy as np
import os

#paths
image_folder = 'c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/maximo'
video_path="c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/videos/maxAndMate222.mp4"

#List's
images = []
class_names = []
myList = os.listdir(image_folder)
#variables 
orb = cv.ORB_create(nfeatures = 1000)

#Loops
for cl in myList: 
    current_image = cv.imread(f"{image_folder}/{cl}", 0)
    images.append(current_image)
    class_names.append(os.path.splitext(cl)[0])

#variable
cap = cv.VideoCapture(video_path)

#fuctions
def findDescription(imageList):  #"""this function is used to go through each element (image) of our asset folder and get the descriptions"""
    descriptiorList = []
    for img in imageList:
        kp, des = orb.detectAndCompute(img, None)
        descriptiorList.append(des)
    return descriptiorList

def findID(img, descriptionList, thres=15):
    kp2, des2 = orb.detectAndCompute(img, None)
    bf = cv.BFMatcher()
    matchList = []
    finalValue = -1
    try:
        for current_des in descriptiorList:
            matches = bf.knnMatch(current_des, des2, k=2)
            good = []
            for m, n in matches:
                if m.distance < 0.75*n.distance:
                    good.append([m])
            matchList.append(len(good))
    except:
        pass
    #print(matchList)
    if len(matchList)!= 0:
        if max(matchList) > thres:
            finalValue = matchList.index(max(matchList))
    return finalValue


#variable2
descriptiorList = findDescription(images)



#main
while True:
    success, img2 = cap.read()
    original_img = img2.copy()
    img2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)

    id = findID(img2, descriptiorList)
    if id != -1:
        cv.putText(original_img, class_names[id], (50,50), cv.FONT_HERSHEY_COMPLEX, 1, (0,69,255), 2)

    cv.imshow("img2", original_img)
    if cv.waitKey(10) & 0xFF == ord('q'):
        break