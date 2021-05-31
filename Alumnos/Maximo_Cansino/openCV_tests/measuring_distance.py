import cv2 as cv

cap = cv.VideoCapture(0)
while True:
    success, image = cap.read()
    cv.imshow("frame", image)
    if cv.waitKey(1) == ord("q"):
        break
cap.release()
cv.destroyAllWindows()
print("END")