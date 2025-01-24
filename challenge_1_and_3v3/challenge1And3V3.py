# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 21:12:36 2019

@author: chris
"""

import cv2 as cv
import cv2
import numpy as np
import serial
from time import sleep
import os
import math
import threading
import time

properties=[
            "CAP_PROP_BRIGHTNESS",# Brightness of the image (only for cameras).
            "CAP_PROP_CONTRAST",# Contrast of the image (only for cameras).
            "CAP_PROP_SATURATION",# Saturation of the image (only for cameras).
            "CAP_PROP_GAIN",# Gain of the image (only for cameras).
            "CAP_PROP_EXPOSURE"]



def TestFrame():
    cap1 = cv2.VideoCapture(1)
    
    brightness=98
    cap1.set(cv2.CAP_PROP_BRIGHTNESS,brightness)

    contrast=128
    cap1.set(cv2.CAP_PROP_CONTRAST,contrast)

    saturation=208
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
        elif key == ord('g'):#白平衡
            gain += 5
            cap1.set(cv2.CAP_PROP_GAIN,gain)
        elif key == ord('f'):
            gain -= 5
            cap1.set(cv2.CAP_PROP_GAIN,gain)
        elif key == ord('b'):#亮度
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

centroid_x=None
centroid_y=None
centroid_x1=None
centroid_y1=None
centroid_x2=None
centroid_y2=None
centroid_xd1=None
centroid_yd1=None
centroid_xd2=None
centroid_yd2=None
true=None

def ShowFrame():
    global centroid_x
    global centroid_y
    global centroid_x1
    global centroid_y1
    global centroid_x2
    global centroid_y2
    global centroid_xd1
    global centroid_yd1
    global centroid_xd2
    global centroid_yd2
    global true
    
    true = 1
    cap = cv2.VideoCapture(1)
    brightness=98
    cap.set(cv2.CAP_PROP_BRIGHTNESS,brightness)

    contrast=128
    cap.set(cv2.CAP_PROP_CONTRAST,contrast)

    saturation=208
    cap.set(cv2.CAP_PROP_SATURATION,saturation)

    gain=103
    cap.set(cv2.CAP_PROP_GAIN,gain)

    exposure=-7
    cap.set(cv2.CAP_PROP_EXPOSURE,exposure)
    while(1):
        
        ret, img = cap.read()
    #=================================Green Ball=================================#
        blur = cv2.GaussianBlur(img, (5,5),0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        up1 = np.array([35,255,230])
        lower1 = np.array([25,230,150])
        mask = cv2.inRange(hsv, lower1, up1)
        bmask = cv2.GaussianBlur(mask, (5,5),0)
        #print('green=>',up1,lower1)
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
    #================================進攻後面色塊=================================#
        
        up2 = np.array([3,255,240])
        lower2 = np.array([0,230,150])
        mask1 = cv2.inRange(hsv, lower2, up2)
        bmask1 = cv2.GaussianBlur(mask1, (5,5),0)
        #print('Front=>',up2,lower2)
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
            cv2.circle(img, ctr1, 4, (0,0,0))
            #cv2.imshow('test2',bmask1)
    #=================================進攻前面色塊=================================#
        up3 = np.array([95,255,200])
        lower3 = np.array([80,230,150])
        mask2 = cv2.inRange(hsv, lower3, up3)
        bmask2 = cv2.GaussianBlur(mask2, (5,5),0)
        #print("Behind=>",up3,lower3)
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
            cv2.circle(img, ctr2, 4, (0,0,0))
            #cv2.imshow('test3',bmask2)
        #=================================守門員色塊=============================#
        
        upd1=np.array([170,223,250])
        lowerd1=np.array([160,135,200])
        maskd1 = cv2.inRange(hsv, lowerd1, upd1)
        bmaskd1 = cv2.GaussianBlur(maskd1, (5,5),0)
        #print('Front=>',up2,lower2)
        centroid_xd1, centroid_yd1 = None, None
        cntsd1=cv2.findContours(bmaskd1.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cntsd1)>0:
            cd1=max(cntsd1,key=cv2.contourArea)
            momentsd1 = cv2.moments(cd1)
            md01 = momentsd1['m00']
            if md01 != 0:
                centroid_xd1 = int(momentsd1['m10']/md01)
                centroid_yd1 = int(momentsd1['m01']/md01)
        ctrd1 = (-1,-1)
        if centroid_xd1 != None and centroid_yd1 != None :
            ctrd1 = (centroid_xd1, centroid_yd1)
            cv2.circle(img, ctrd1, 4, (0,0,0))
        #=================================第一道防線色塊=============================#
        upd2 = np.array([20,170,250])
        lowerd2 = np.array([15,100,150])
        maskd2 = cv2.inRange(hsv, lowerd2, upd2)
        bmaskd2 = cv2.GaussianBlur(maskd2, (5,5),0)
        #print('Front=>',up2,lower2)
        centroid_xd2, centroid_yd2 = None, None
        cntsd2=cv2.findContours(bmaskd2.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cntsd2)>0:
            cd2=max(cntsd2,key=cv2.contourArea)
            momentsd2 = cv2.moments(cd2)
            md02 = momentsd2['m00']
            if md02 != 0:
                centroid_xd2 = int(momentsd2['m10']/md02)
                centroid_yd2 = int(momentsd2['m01']/md02)
        ctrd2 = (-1,-1)
        if centroid_xd2 != None and centroid_yd2 != None :
            ctrd2 = (centroid_xd2, centroid_yd2)
            cv2.circle(img, ctrd2, 4, (0,0,0))
            #return centroid_xd2
        #=================================顯示畫面=================================#
        
        cv2.circle(img,(13,165),8,(255,255,0),-1)
        cv2.imshow('image',img)
        #sleep(0.3)
        key=cv2.waitKey(20)
        if key == 27:
            true = 0 
            break
      
    cap.release()
    cv2.destroyAllWindows()
    
def Check():
    print(true)
    
    while(true==1):
        
        tStart = time.time()#計時開始
 

        time.sleep(2)
        for x in range(1000):
            x += 1
        print(x)
            
 
        tEnd = time.time()
        
        print ("It cost %f sec" % (tEnd - tStart))
        print (tEnd - tStart)
    

def Attack1():
    #print(true)
    cg=0
    ct=0
    rg=0
    pt=0
    rt=0
    pg=0
    gt=0
    minct=0
    maxct=0
    minrg=0
    maxrg=0
    minrt=0
    maxrt=0
    mingt=0
    maxgt=0
    pr=0
    s=serial.Serial("com3",57600)
    while(true == 1):
        #print('ya')
        if not centroid_x1 is None and not centroid_y1 is None and not centroid_x2 is None and not centroid_y2 is None:
            ball = np.array([centroid_x , centroid_y])
            if not ball[0] is None and not ball[1] is None:
                Green = np.array([ball[0] , ball[1]])#球
            Target = np.array([13,165])#球門
            Red = np.array([centroid_x1 , centroid_y1])#進攻前面
            purple= np.array([centroid_x2 , centroid_y2])#進攻後面
            
            #print("Green=>",Green)
            #print("Target=>",Target)
            #print("Red=>",Red)
            #print("purple=>",purple)

            if not Green[0] is  None and not Red[0] is None and not purple[0] is None:
                centerx=int((purple[0]+Red[0])/2)
                centery=int((purple[1]+Red[1])/2)
                Central=np.array([centerx,centery])
                lent=abs(centerx-Green[0])+abs(centery-Green[1])
    #=================================定義斜率=================================#
                if float(centerx-Green[0])!=0 and float(centery-Green[1])!=0:
                    cg=float(centery-Green[1])/float(centerx-Green[0])
                    #print('Central&Green=',cg)
                if float(centerx-Target[0])!=0 and float(centery-Target[1])!=0:
                    ct=float(centery-Target[1])/float(centerx-Target[0])
                    #print('Cetral&Target=',ct)
                if float(Red[1]-Green[1])!=0 and float(Red[0] - Green[0])!=0:#Front and ball
                    rg=float(Red[1]-Green[1])/float(Red[0] - Green[0])
                    #print('rg=>',rg)
                if float(purple[1]-Target[1])!=0 and float(purple[0] - Target[0])!=0:  #Behind and Target
                    pt=float(purple[1]-Target[1])/float(purple[0] - Target[0])
                    #print('pt=>',pt)
                if float(Red[1]-Target[1])!=0 and float(Red[0] - Target[0])!=0:   #Front and Target
                    rt=float(Red[1]-Target[1])/float(Red[0] - Target[0])
                    #print('rt=>',rt)
                if float(purple[1]-Green[1])!=0 and float(purple[0] - Green[0])!=0:  #Behind and ball
                    pg=float(purple[1]-Green[1])/float(purple[0] - Green[0])
                    #print('pg=>',pg)
                if float(Green[1]-Target[1])!=0 and float(Green[0] - Target[0])!=0:
                    gt = float(Green[1]-Target[1])/float(Green[0]-Target[0])
                    #print("gt=>",gt)
                if (float(purple[1]-Red[1])!=0 and float(purple[0]-Red[0])!=0):
                    pr=float(purple[1]-Red[1])/float(purple[0]-Red[0])
                    print("pr=>",pr)
                    n=abs((1/(Red[0] - Green[0])*1.8))#
                    m=abs(pt-rt)*10#error
                    error=0.25
                    
                    #print("n:",n)
                    #print("m:",m)
                    #minrg=rg-2*n
                    #maxrg=rg+2*n
                    #print("maxrg=>",maxrg)
                    #print("minrg=>",minrg)
                    minct=ct-error
                    maxct=ct+error
                    print("maxct=>",maxct)
                    print("minct=>",minct)
                    mingt=gt-0.15
                    maxgt=gt+0.15
                    print("lent:",lent)
                    print("maxgt=>",maxgt)
                    print("mingt=>",mingt)
                    #print("lent=>",lent)
    #====================================進攻邏輯======================================#
        sleep(0.5)
        if pr<minct:
            print("turn right")
            while True:
                    
                s.write('Cd'.encode())
                #s.write('Cd'.encode())
                sleep(1)
                    
                break
        elif pr>maxct:
            print("turn left")
                
            while True:
                    
                s.write('Ca'.encode())
                sleep(1)
                    
                break
        elif pr<maxct and pr>minct:
            if lent >35:#30
                if cg>maxgt:
                    print("go right")
                    while True:  
                    
                        s.write('Cg'.encode())#往右橫移，會往左邊偏
                        sleep(0.7)
                    
                    
                        
                        break
                elif cg<mingt:
                    print("go left")
                    
                    while True:
                      
                        s.write('Cf'.encode())
                        sleep(0.7)
                        
                    
                        break
                elif (cg>mingt and cg<maxgt):
                    print("Straight")
                    
                    while True:  
                
                        s.write('Cw'.encode())
                        sleep(0.5)
                
                        break      
            if lent <=35:
                if (pr>mingt and pr<maxgt):
                        print("kick")
                        while True:
                            s.write('Cg'.encode())#下lent30#中lent25
                            sleep(0.5)
                            s.write('Cg'.encode())
                            sleep(0.5)
                            s.write('Cg'.encode())
                            sleep(0.5)
                            #s.write('Cf'.encode())
                            #sleep(0.5)
                            #s.write('Cf'.encode())
                            #sleep(0.5)
                            #s.write('Cf'.encode())
                            #sleep(0.5)
                            #s.write('Ca'.encode())
                            #sleep(0.5)
                            #s.write('Cw'.encode())
                            #sleep(1)
                            s.write('Cw'.encode())
                            sleep(1)
                            s.write('Cj'.encode())
                            sleep(1)
                            break
                elif pr<mingt:
                    print("turn right")
                    while True:  
                    
                        s.write('Cd'.encode())
                        sleep(1)
                        break
                
                elif pr>maxgt:
                    print("turn left")
                
                    while True:
                    
                        s.write('Ca'.encode())
                        sleep(1)
                    
                        break

DF2=0 
L_1=0
R_1=0           
mutex = threading.Lock()
def defense():
    global DF2
    global L_1
    global R_1
    s=serial.Serial("com3",57600)
    while(true == 1):
        if not centroid_x is  None and not centroid_y is None and not centroid_xd1 is None and not centroid_yd1 is None and not centroid_xd2 is None and not centroid_yd2 is None:
            Green = np.array([centroid_x , centroid_y])#球
            DFront1 = np.array([centroid_xd2 , centroid_yd2])#防守者前面那隻
            DFront = np.array([centroid_xd1 , centroid_yd1])#守門員色塊
            #DBehind = np.array([centroid_xd2 , centroid_yd2])#防守後面
            PointLow = np.array([301,382],dtype='float16')#第二道防線second座標
            PointHigh = np.array([305,99],dtype='float16')#第二道防線first座標
            PointLow1 = np.array([445,306],dtype='float16')#第一道防線second座標
            PointHigh1 = np.array([440,181],dtype='float16')#第一道防線first座標
              
            midy = (PointLow[1]+PointHigh[1])/2
        #=================================================defense2================================
            if Green[0]>=PointLow1[0]:
                
                if DFront1[1]<PointHigh1[1]:
                    print("DefendLeft2")
                    while True:
                    
                        s.write('Bf'.encode())
                        sleep(1)
                        s.write('Ba'.encode())
                        sleep(1)
                        #s.write('Aa'.encode())
                        #sleep(1)
                        break
                elif DFront1[1]>PointLow1[1]:
                    print("DefendRight2")
                    while True:
                        s.write('Bg'.encode())
                        #s.write('Bd'.encode())
                        sleep(1)
                        #mutex.acquire()
                        DF2+=1
                        #mutex.release()
                        break
                elif Green[1]>=DFront1[1]:
                    print("DefendLeft 2")
                    while True:
                        
                        s.write('Bf'.encode())
                        sleep(1)
                        s.write('Ba'.encode())
                        sleep(1)
                        #s.write('Ba'.encode())
                        #sleep(1)
                        
                        break
                elif Green[1]<DFront1[1]:
                    print("DefendRight 2")
                    while True:
                        
                        s.write('Bg'.encode())
                        #s.write('Bg'.encode())
                        #s.write('Bd'.encode())
                        #mutex.acquire()
                        DF2+=1
                        #mutex.release()
                        
                        sleep(2)
                            
                        break
                else:
                    print('fall')
            elif Green[0]<PointLow1[0]:
                if DFront[1]>(midy+10):
                    print("Go right 2")
                    while True:
                
                        s.write('Bg'.encode())
                        #s.write('Bg'.encode())
                        #s.write('Bd'.encode())
                        #mutex.acquire()
                        DF2+=1
                        #mutex.release()
                        sleep(1)
                    
                        break
                elif DFront1[1]<(midy-10):
                    print("Go left 2")
                    while True:  
                    
                        s.write('Bf'.encode())
                        sleep(1)
                        s.write('Ba'.encode())
                        sleep(1)
                        #s.write('Ba'.encode())
                        #sleep(1)
                
                        break
                elif (DFront1[1]>(midy-10)) and (DFront1[1]<(midy+10)):
                    print("Stand 2")
                    while True:
                    
                        s.write(''.encode())
                        sleep(1)
                    
                        break  
                else:
                    print('fall')
            else:
                print('fall')
            
            if DF2>=2:
                while True:
                    
                    print("turn right")
                    print("DF2:",DF2)
                    s.write('Bd'.encode())
                    sleep(1)
                    mutex.acquire()
                    DF2=0
                    mutex.release()
                    break
            
    #=================================================defense2================================
    #==============================================Defense================================================#
           
            if Green[0]>=PointLow[0]:
                if DFront[1]<PointHigh[1]:
                    print("DefendLeft")
                    while True:
                    
                        s.write('Af'.encode())
                        #s.write('Aa'.encode())
                        L_1+=1
                        sleep(1)
                        break
                elif DFront[1]>PointLow[1]:
                    print("DefendRight")
                    while True:
                    
                        s.write('Ag'.encode())
                        #s.write('Bd'.encode())
                        R_1+=1
                        sleep(1)
                        
                        break
                elif Green[1]>=DFront[1]:
                    print("DefendLeft")
                    while True:
                        
                        s.write('Af'.encode())
                        #s.write('Aa'.encode())
                        L_1+=1
                        sleep(1)
                        
                        break
                elif Green[1]<DFront[1]:
                    print("DefendRight")
                    while True:
                        
                        s.write('Ag'.encode())
                        #s.write('Bd'.encode())
                        R_1+=1
                        sleep(1)
                            
                        break
                else:
                    print('fall')
            elif Green[0]<PointLow[0]:
                if DFront[1]>(midy+10):
                    print("Go right")
                    while True:
                
                        s.write('Ag'.encode())
                        #s.write('Bd'.encode())
                        R_1+=1
                        sleep(1)
                    
                        break
                elif DFront[1]<(midy-10):
                    print("Go left")
                    while True:  
                    
                        s.write('Af'.encode())
                        #s.write('Aa'.encode())
                        L_1+=1
                        sleep(1)
                
                        break
                elif (DFront[1]>(midy-10)) and (DFront[1]<(midy+10)):
                    print("Stand")
                    while True:
                    
                        s.write(''.encode())
                        sleep(1)
                    
                        break
                else:
                    print('fall')
            if R_1>=2:
                while True:
                    
                    print("turn right")
                    print("R_1:",R_1)
                    s.write('Ad'.encode())
                    sleep(1)
                    #mutex.acquire()
                    R_1=0
                    #mutex.release()
                    break
            if L_1>=2:
                while True:
                    
                    print("turn left")
                    print("L_1:",L_1)
                    s.write('Aa'.encode())
                    sleep(1)
                    s.write('As'.encode())
                    sleep(1)
                    #mutex.acquire()
                    L_1=0
                    #mutex.release()
                    break
            
        
        
    
def main():
    TestFrame()
    #ShowFrame()
    #Attack1()
    t=threading.Thread(target=ShowFrame) 
    t1 = threading.Thread(target=Attack1)#用法和Process类很相似
    #t3=threading.Thread(target=defense)
    t2 = threading.Thread(target=defense)
    t.start()
    t1.start()
    #t1.start()
    t2.start()
    t.join()
    #t3.join()
    t1.join()
    t2.join()
    print(true)
    #t2.join()     
    
        
if __name__ == '__main__':
    main()        
        

    