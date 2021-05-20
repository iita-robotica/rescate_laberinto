import cv2 as cv

def showing(img):
    return cv.imshow("output", img)

img= cv.imread("openCV/assets/images/warning_webots_icons.png")
showing(img)
cv.waitKey(0)
canny=cv.Canny(img,100,100)
showing(canny)
cv.waitKey(0)
size = img.shape
print(size)
imgResized = cv.resize(img, (size[0]+500, size[1]+500))
showing(imgResized)
cv.waitKey(0)
print(imgResized.shape)

 