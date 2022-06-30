import numpy as np
import cv2


cap = cv2.VideoCapture("/home/ibrahim/sualtı/sualti videolari/filename5.avi")

while True:
    _, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    gray_img=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    img	= cv2.medianBlur(gray_img,	5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
     
    #center
     
    circles	= cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,120,param1=100,param2=30,minRadius=0,maxRadius=0)
  
     

    if circles is not None:
         circles = np.uint16(np.around(circles))
         for i in circles[0, :]:
             cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),6)
             cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
             print("x Kordinatı : " + str(i[0])) 
             print("Y Kordinatı : " + str(i[1])) 
             break

    cv2.imshow("Kirmizi Bulunmus Hali", frame)
    key = cv2.waitKey(33)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
