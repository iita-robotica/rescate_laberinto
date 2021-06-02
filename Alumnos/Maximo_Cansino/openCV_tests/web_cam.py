import cv2 as cv
frame_width = 480
frame_heigh = 640
cap = cv.VideoCapture(0)
cap.set(3, frame_width)
cap.set(4, frame_heigh)
cap.set(10,130)
while True:
    success, img = cap.read()
    img = cv.rotate(img, cv.ROTATE_180)
    cv.imshow("result", img) 
    if cv.waitKey(1) & 0xFF == ord("q"):
        break