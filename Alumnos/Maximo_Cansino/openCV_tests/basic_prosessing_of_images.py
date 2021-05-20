import cv2 as cv
import numpy as np


#resize an image:


img = np.zeros((600,600, 3),np.uint8)
#img[300,300]= 0,255,0 #color a node of the grid

""" #to paint a specifict part of the matrix

img[
    
    1:300,#y
    1:300 #x
] = 0,0,255 #Blue, red, green
 """
#cv.line(img,(0,0),(300,300), (0,255,0), 5)


""" #cv.rectangle(img, (0,0), (300,300), (0,255,0), cv.FILLED)
HelloCircle =cv.circle(img, (img.shape[1]//2, img.shape[0]//2), 50, (255,255,255), 2)
cv.rectangle(HelloCircle, (0,0), (300,300), (0,0,255), 3)
print(f"/////\n{img.shape}\n///////") """
cv.putText(img, "Hello World",(100,300),cv.FONT_HERSHEY_TRIPLEX, 5.0, (0,69,255), 1)
cv.imshow("matrix", img)
cv.waitKey(0)