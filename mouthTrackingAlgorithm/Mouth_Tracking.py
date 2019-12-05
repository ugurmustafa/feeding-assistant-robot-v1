# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2
import numpy as np

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
mouth_cascade = cv2.CascadeClassifier('haarcascade_mcs_mouth.xml')

cap = cv2.VideoCapture(0)
ds_factor = 0.5
while True:
    ret,img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    height = np.size(img, 0)
    width = np.size(img, 1)
    #print(width,height)
    #cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
    faces = face_cascade.detectMultiScale(gray,1.3,5)

    
    for(x,y,w,h) in faces:
        #○cv2.rectangle(img, (x,y),(x+w, y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        mouths = mouth_cascade.detectMultiScale(roi_gray)
        eyes = eye_cascade.detectMultiScale(roi_gray)
        
        #for(ex,ey,ew,eh) in eyes:
            #cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        for(mx,my,mw,mh) in mouths:
            #my = int(my - 0.15*mh)
            if(my*mx >15000):
                roi_color_2 = roi_color[my:my+mh,mx:mx+mw]
                rows,cols,_ = roi_color_2.shape
                roi_gray_2 = cv2.cvtColor(roi_color_2, cv2.COLOR_BGR2GRAY)
                roi_gray_2 = cv2.GaussianBlur(roi_gray_2,(7,7),0)
                hsv = cv2.cvtColor(roi_color_2, cv2.COLOR_BGR2HSV)
                #cv2.imshow('img',hsv)
                lower_red = np.array([0,50,50])
                upper_red = np.array([15,255,255])
                mask0 = cv2.inRange(hsv, lower_red, upper_red)

                lower_red2 = np.array([160,50,50])
                upper_red2 = np.array([180,255,255])
                mask1 = cv2.inRange(hsv, lower_red2, upper_red2)
    
                mask = mask0+mask1
                mask = cv2.inRange(hsv, lower_red, upper_red)
                #res = cv2.bitwise_and(roi_color_2,roi_color_2, mask= mask)
                #cv2.rectangle(mask,(mx,my),(mx+mw,my+mh),(0,255,255),2)
                cv2.imshow('mask',mask)
                #rows,cols,_ = roi_color_2.shape
                #_,threshold = cv2.threshold(roi_gray_2,60,255,cv2.THRESH_BINARY)
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = sorted(contours,key=lambda x: cv2.contourArea(x),reverse = True)
                for cnt in contours:
                    #cv2.drawContours(roi_color_2,[cnt],-1,(0,0,255),3)
                    (cx, cy, cw, ch) = cv2.boundingRect(cnt)          
                    cv2.rectangle(roi_color_2,(cx,cy),(cx+cw,cy+ch),(0,255,255),2)
                    cv2.putText(roi_color, 'Mouth', (mx+mw, my+mh), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,cv2.LINE_AA)
                    """cv2.line(roi_color_2,(cx+int(cw/2),0),(cx+int(cw/2),int(rows/2)),(255,255,255),1)
                    cv2.line(roi_color_2,(0,cy+int(ch/2)),(int(cols/2),cy+int(ch/2)),(255,255,255),1)
                    cv2.putText(roi_color,'Y = '+ str(mh/2), (mx+int(8*mw/15), my+int(mh/4)), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 255, 0), 1,cv2.LINE_AA)
                    cv2.putText(roi_color,'X = '+ str(mw/2), (mx+int(mw/12), my+int(2*mh/3)), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 255, 0), 1,cv2.LINE_AA)"""
                    #print(mh/2)
                    cv2.line(img,(320,240),(320,y+my+int(mh/2)),(255,70,70),3) #Ağıza olan dikey uzaklık çizgisi
                    cv2.line(img,(320,240),(x+mx+int(mw/2),240),(255,70,70),3) #Ağıza olan yatay uzaklık çizgisi
                    
                   
                    cv2.putText(img,'deltaX = '+  str(320-(x+mx+int(mw/2))), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1,cv2.LINE_AA)
                    cv2.putText(img,'deltaY = '+  str(240-(y+my+int(mh/2))), (20,60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1,cv2.LINE_AA)
                    #cv2.circle(img,(int(width/2),int(height/2)),3,(0,255,255),0)
                    break
                break
                #break
    
    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xFF
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()