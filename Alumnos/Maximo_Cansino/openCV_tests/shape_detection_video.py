import cv2 as cv
import numpy as np

def getContours(img, img_copy):
    objetc_type=""
    contours, hierarchy = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for find_contours in contours:
        area = cv.contourArea(find_contours)
        print(area)
        if area > 500:
            cv.drawContours(img_copy, find_contours, -1, (0,0,255), 3)
            perimetter = cv.arcLength(find_contours, True)
            #print (f"Perimetter: {perimetter}")
            approx= cv.approxPolyDP(find_contours, 0.02*perimetter, True) 
            print(f"Corners approx: {len(approx)}")
            objCor=len(approx)
            """let's draw do bounding box"""
            x, y, w, h = cv.boundingRect(approx)
            if objCor == 3:
                objetc_type = "Triangule"
            elif objCor == 4:
                # aspRadio = w/float(h)
                # print(f"Aspecr Radio == {aspRadio}")
                objetc_type == "Square"
            else: 
                objetc_type="null"
            cv.rectangle(img_copy, (x,y), (x+w, y+h), (0,255,0), 3)
            cv.putText(img_copy,objetc_type,(x+(w//2),y+(h//2)), cv.FONT_HERSHEY_COMPLEX, 0.9, (0,0,0), 1)


image_path="c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/videos/test.mp4"
img= cv.VideoCapture(image_path)
#img= cv.resize(img,(500,500))

img_gray=cv.cvtColor(img, cv.COLOR_BGR2GRAY)
img_blur=cv.GaussianBlur(img_gray, (7,7),1)

#edge detection
imgBlack = np.zeros_like(img)
canny = cv.Canny(img_blur, 50, 50, 2)
img_copy= img.copy()
getContours(canny, img_copy)

while True:
    success, img_frame = img.read()
    #img_frame = cv.rotate(img_frame, cv.ROTATE_180)
    img_gray=cv.cvtColor(img_frame, cv.COLOR_BGR2GRAY)
    img_blur=cv.GaussianBlur(img_gray, (7,7),1)

    #edge detection
    imgBlack = np.zeros_like(img_frame)
    canny = cv.Canny(img_blur, 50, 50, 2)
    img_copy= img_frame.copy()
    getContours(canny, img_copy)

    cv.imshow("result", img_frame) 
    if cv.waitKey(1) & 0xFF == ord("q"):
        break





""" cv.imshow("img", img)
# cv.imshow("gray", img_gray)
# cv.imshow("blur", img_blur)
# cv.imshowimg ", img_blur)
cv.imshow("canny", canny)
cv.imshow("filter", imgBlack)
cv.imshow("img copy", img_copy)

cv.waitKey(0) """