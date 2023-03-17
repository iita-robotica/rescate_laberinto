from controller import Robot, Camera
import cv2  as cv
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera1 = robot.getDevice("camera1")
camera1.enable(timestep)
camera2 = robot.getDevice("camera2")
camera2.enable(timestep)

def empty(a):
    pass

def vconcat_different_size_images(im_list, interpolation=cv.INTER_CUBIC):
    w_min = min(im.shape[1] for im in im_list)
    im_list_resize = [cv.resize(im, (w_min, int(im.shape[0] * w_min / im.shape[1])), interpolation=interpolation)
                      for im in im_list]
    return cv.vconcat(im_list_resize)

cv.namedWindow("trackBars")
cv.resizeWindow("trackBars", 640,240)
cv.createTrackbar("hue mini", "trackBars", 0, 255, empty),
cv.createTrackbar("hue max", "trackBars", 255, 255, empty),
cv.createTrackbar("saturation mini", "trackBars", 0, 255, empty),
cv.createTrackbar("saturation max", "trackBars", 255, 255, empty),
cv.createTrackbar("min value", "trackBars", 0 , 255, empty),
cv.createTrackbar("max value", "trackBars", 255, 255, empty),



while robot.step(timestep) != -1:
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    image1 = np.array(image1 ,dtype=np.uint8)
    
    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    image2 = np.array(image2 ,dtype=np.uint8)
    
    hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGRA2BGR)
    hsv_image2 = cv.cvtColor(image2, cv.COLOR_BGRA2BGR)
    hue_min=cv.getTrackbarPos("hue mini", "trackBars")
    hue_max=cv.getTrackbarPos("hue max", "trackBars")

    saturation_min=cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max=cv.getTrackbarPos("saturation max", "trackBars")
    
    min_value=cv.getTrackbarPos("min value", "trackBars")
    max_value=cv.getTrackbarPos("max value", "trackBars")

    #print(f"The min hue is -> {hue_min}\tThe max hue is -> {hue_max}\tThe min saturation is -> {saturation_min}\tThe max saturation is -> {saturation_max}\tThe min value is -> {min_value}\tThe max value is -> {max_value}")

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])
    
    mask1 = cv.inRange(hsv_image1, lower, upper)
    mask2 = cv.inRange(hsv_image2, lower, upper)
    imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)
    imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)
    
    #cv.imshow("BGR", image1)
    #cv.imshow("HSV", hsv_image1) 
    # cv.imshow("Mask1", mask1)
    # cv.imshow("Mask2", mask2)

    #cv.imshow("Final Reslut", imgResult1)
    both_mask_images = cv.hconcat([mask1, mask2])
    both_final_results = cv.hconcat([imgResult1, imgResult2])
    rows_list = [both_mask_images, both_final_results]
    #print(f"Mask row --> {both_mask_images.size}, and FResult row size --> {both_final_results.size}")
    #camera_panel = cv.vconcat([both_mask_images,    both_final_results])
    #cv.imshow("bmasks", both_mask_images)
    #rows = vconcat_different_size_images(rows_list)
    cv.imshow("Camera panel masks", both_mask_images)
    cv.imshow("Camera panel final results", both_final_results)
    cv.waitKey(1)