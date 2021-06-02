import cv2
import pytesseract
img =cv2.imread("Hellofriendd.jpg")   #here we read the images
img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # we must to apply this to better results
#print (pytesseract.image_to_string(img))    
lisA=[]   #in this list we store the lectures of each letter


##Detecting characters
hImg,wImg,_ = img.shape
boxes = pytesseract.image_to_boxes(img)
for b in boxes.splitlines():
    #print (b)
    b=b.split(" ")
    print (b)
    lisA.append(b)
    x,y,w,h =int(b[1]),int(b[2]),int(b[3]),int(b[4])
    cv2.rectangle(img, (x,hImg-y), (w,hImg-h), (0,0,255),1)
    cv2.putText(img,b[0], (x,hImg-y+10), cv2.FONT_HERSHEY_COMPLEX,1,(50,50,255),2)
cv2.imshow("result", img)
cv2.waitKey(0)


for i in range(len(lisA)):
    for i in lisA[i]:
        if i == "H":
            print ("\nI found a victim -->  H")
        if i == "S":
            print ("\nI found a victim --> S")
        if i == "U":
            print ("\nI found a victim --> U" )
            
            
