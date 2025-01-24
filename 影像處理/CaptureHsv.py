# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 00:01:36 2019

@author: chris
"""
import numpy as np
import cv2

def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,up,lower

    maxhsv=np.array([0 , 0, 0])
    minhsv=np.array([255 , 255, 255])
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy=x,y

    elif event==cv2.EVENT_MOUSEMOVE and flags==cv2.EVENT_FLAG_LBUTTON:
        if drawing == True:
            cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)

    elif event ==cv2.EVENT_LBUTTONUP:
        drawing == False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for i in range (ix,x):
            for j in range (iy,y):
                if hsv[j][i][0]>maxhsv[0]:
                    maxhsv[0]=hsv[j][i][0]
                if hsv[j][i][1]>maxhsv[1]:
                    maxhsv[1]=hsv[j][i][1]
                if hsv[j][i][2]>maxhsv[2]:
                    maxhsv[2]=hsv[j][i][2]
                if hsv[j][i][0]<minhsv[0]:
                    minhsv[0]=hsv[j][i][0]
                if hsv[j][i][1]<minhsv[1]:
                    minhsv[1]=hsv[j][i][1]
                if hsv[j][i][2]<minhsv[2]:
                    minhsv[2]=hsv[j][i][2]
        up=maxhsv
        lower=minhsv
        print(x,y)

properties=[
            "CAP_PROP_BRIGHTNESS",# Brightness of the image (only for cameras).
            "CAP_PROP_CONTRAST",# Contrast of the image (only for cameras).
            "CAP_PROP_SATURATION",# Saturation of the image (only for cameras).
            "CAP_PROP_GAIN",# Gain of the image (only for cameras).
            "CAP_PROP_EXPOSURE"]

cap1 = cv2.VideoCapture(1)

brightness=98
cap1.set(cv2.CAP_PROP_BRIGHTNESS,brightness)

contrast=128
cap1.set(cv2.CAP_PROP_CONTRAST,contrast)

saturation=188
cap1.set(cv2.CAP_PROP_SATURATION,saturation)

gain=103
cap1.set(cv2.CAP_PROP_GAIN,gain)

exposure=-7
cap1.set(cv2.CAP_PROP_EXPOSURE,exposure)    
while(True):
    # Capture frame-by-frame
    ret, frame = cap1.read()
    # Our operations on the frame come here
    #rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb=frame
    print ("\n\n")
    for prop in properties:
        val=cap1.get(eval("cv2."+prop))
        print (prop+": "+str(val))
    # Display the resulting frame
    cv2.imshow('frame12',rgb)
    key=cv2.waitKey(1)
    if key == 27:
        break
    elif key == ord('g'):#白平衡，不變
        gain += 5
        cap1.set(cv2.CAP_PROP_GAIN,gain)
    elif key == ord('f'):
        gain -= 5
        cap1.set(cv2.CAP_PROP_GAIN,gain)
    elif key == ord('b'):#亮度，不變
        brightness+=5
        cap1.set(cv2.CAP_PROP_BRIGHTNESS,brightness)
    elif key == ord('v'):
        brightness-=5
        cap1.set(cv2.CAP_PROP_BRIGHTNESS,brightness)
    elif key == ord('c'):#對比
        contrast+=5
        cap1.set(cv2.CAP_PROP_CONTRAST,contrast)
    elif key == ord('x'):
        contrast-=5
        cap1.set(cv2.CAP_PROP_CONTRAST,contrast)
    elif key == ord('s'):#飽和
        saturation+=5
        cap1.set(cv2.CAP_PROP_SATURATION,saturation)
    elif key == ord('a'):
        saturation-=5
        cap1.set(cv2.CAP_PROP_SATURATION,saturation)
    elif key == ord('e'):#曝光度
        exposure+=1
        cap1.set(cv2.CAP_PROP_EXPOSURE,exposure)
    elif key == ord('w'):
        exposure-=1
        cap1.set(cv2.CAP_PROP_EXPOSURE,exposure)

cv2.destroyAllWindows()



while(1):
    cap = cv2.VideoCapture(1)
    img = np.zeros((480,640,3),np.uint8)#(480,640)為(0)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',draw_circle)
    while(1):
        ret, frame = cap.read()
        frame1=cv2.addWeighted(frame,1,img,1,0)
        #cv2.imshow('image',img)
        cv2.imshow('frame',frame)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    up1=up
    lower1=lower
    print('green=>',up,lower)
    
    drawing = False
    ix,iy = -1,-1
    img = np.zeros((480,640,3),np.uint8)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',draw_circle)
    while(1):
        ret, frame = cap.read()
        frame1=cv2.addWeighted(frame,1,img,1,0)
        #    cv2.imshow('image',img)
        cv2.imshow('frame',frame1)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    up2=up
    lower2=lower
    print('red=>',up,lower)
    
    drawing = False
    ix,iy = -1,-1
    img = np.zeros((480,640,3),np.uint8)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',draw_circle)
    
    
    while(1):
        ret, frame = cap.read()
        frame1=cv2.addWeighted(frame,1,img,1,0)
        #    cv2.imshow('image',img)
        cv2.imshow('frame',frame1)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    up3=up
    lower3=lower
    print('purple=>',up,lower)
    
    drawing = False
    ix,iy = -1,-1
    img = np.zeros((480,640,3),np.uint8)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',draw_circle)
    
    
    while(1):
        ret, frame = cap.read()
        frame1=cv2.addWeighted(frame,1,img,1,0)
        #    cv2.imshow('image',img)
        cv2.imshow('frame',frame1)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    up4=up
    lower4=lower
    print('pink=>',up,lower)
    
    
    drawing = False
    ix,iy = -1,-1
    img = np.zeros((480,640,3),np.uint8)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',draw_circle)
    
    
    while(1):
        ret, frame = cap.read()
        frame1=cv2.addWeighted(frame,1,img,1,0)
        #    cv2.imshow('image',img)
        cv2.imshow('frame',frame1)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    up5=up
    lower5=lower
    print('brown=>',up,lower)
    
    
    while(1):
        ret, img = cap.read()
        blur = cv2.GaussianBlur(img, (5,5),0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower1, up1)
        bmask = cv2.GaussianBlur(mask, (5,5),0)
        centroid_x, centroid_y = None, None
        cnts=cv2.findContours(bmask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts)>0:
            c=max(cnts,key=cv2.contourArea)
            moments = cv2.moments(c)
            m00 = moments['m00']
            if m00 != 0:
                centroid_x = int(moments['m10']/m00)
                centroid_y = int(moments['m01']/m00)
        ctr = (-1,-1)
        if centroid_x != None and centroid_y != None :
            ctr = (centroid_x, centroid_y)
            cv2.circle(img, ctr, 4, (0,0,0))
        #cv2.imshow('test1',bmask)
        mask1 = cv2.inRange(hsv, lower2, up2)
        bmask1 = cv2.GaussianBlur(mask1, (5,5),0)
        centroid_x1, centroid_y1 = None, None
        cnts1=cv2.findContours(bmask1.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts1)>0:
            c1=max(cnts1,key=cv2.contourArea)
            moments1 = cv2.moments(c1)
            m01 = moments1['m00']
            if m01 != 0:
                centroid_x1 = int(moments1['m10']/m01)
                centroid_y1 = int(moments1['m01']/m01)
        ctr1 = (-1,-1)
        if centroid_x1 != None and centroid_y1 != None :
            ctr1 = (centroid_x1, centroid_y1)
            cv2.circle(img, ctr1, 4, (170,150,255))
        #cv2.imshow('test2',bmask1)
        mask2 = cv2.inRange(hsv, lower3, up3)
        bmask2 = cv2.GaussianBlur(mask2, (5,5),0)
        centroid_x2, centroid_y2 = None, None
        cnts2=cv2.findContours(bmask2.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts2)>0:
            c2=max(cnts2,key=cv2.contourArea)
            moments2 = cv2.moments(c2)
            m02 = moments2['m00']
            if m02 != 0:
                centroid_x2 = int(moments2['m10']/m02)
                centroid_y2 = int(moments2['m01']/m02)
        ctr2 = (-1,-1)
        if centroid_x2 != None and centroid_y2 != None :
            ctr2 = (centroid_x2, centroid_y2)
            cv2.circle(img, ctr2, 4, (10,127,94))
            #=============================pink==================================
        mask3 = cv2.inRange(hsv, lower4, up4)
        bmask3 = cv2.GaussianBlur(mask3, (5,5),0)
        centroid_x3, centroid_y3 = None, None
        cnts3=cv2.findContours(bmask3.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts3)>0:
            c3=max(cnts3,key=cv2.contourArea)
            moments3 = cv2.moments(c3)
            m03 = moments3['m00']
            if m03 != 0:
                centroid_x3 = int(moments3['m10']/m03)
                centroid_y3 = int(moments3['m01']/m03)
        ctr3 = (-1,-1)
        if centroid_x3 != None and centroid_y3 != None :
            ctr2 = (centroid_x3, centroid_y3)
            cv2.circle(img, ctr3, 4, (10,127,94))
         #=============================brown==================================
        mask4 = cv2.inRange(hsv, lower5, up5)
        bmask4 = cv2.GaussianBlur(mask4, (5,5),0)
        centroid_x4, centroid_y4 = None, None
        cnts4=cv2.findContours(bmask4.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts4)>0:
            c4=max(cnts4,key=cv2.contourArea)
            moments4 = cv2.moments(c4)
            m04 = moments4['m00']
            if m04 != 0:
                centroid_x4 = int(moments4['m10']/m04)
                centroid_y4 = int(moments4['m01']/m04)
        ctr4 = (-1,-1)
        if centroid_x4 != None and centroid_y4 != None :
            ctr4 = (centroid_x4, centroid_y4)
            cv2.circle(img, ctr4, 4, (10,127,94))
        
        
        
        cv2.imshow('image',img)
        #cv2.imshow('test3',bmask2)
        '''
        Green = np.array([centroid_x , centroid_y])
        Target = np.array([220 , 25])
        Red = np.array([centroid_x1 , centroid_y1])
        purple= np.array([centroid_x2 , centroid_y2])
        
        print("ball",centroid_x,centroid_x)
        print("淺green",centroid_x1,centroid_x1)
        print("red",centroid_x2,centroid_x2)
        print("粉",centroid_x3,centroid_x3)
        print("棕",centroid_x4,centroid_x4)
        '''
        if cv2.waitKey(20) & 0xFF == 27:
            break
    break
cap.release()
cv2.destroyAllWindows()

