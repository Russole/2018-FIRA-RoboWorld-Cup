

import cv2 as cv
import cv2
import numpy as np
import serial
from time import sleep
import os
import math
import threading
import time

forward=[0xFF,0x55,0x01,0xFE,0x00,0xFF]
back=[0xFF,0x55,0x02,0xFD,0x00,0xFF]
right=[0xFF,0x55,0x09,0xF6,0x01,0xFE]
right_s=[0xFF,0x55,0x08,0xF7,0x00,0xFF]
left=[0xFF,0x55,0x05,0xFA,0x01,0xFE]
left_s=[0xFF,0x55,0x04,0xFB,0x00,0xFF]
kick=[0xFF,0x55,0x81,0x7E,0x00,0xFF]
kick_l=[0xFF,0x55,0x21,0xDE,0,0xFF]
fd=[0xFF,0x55,0x11,0xEE,0x00,0xFF]
fu=[0xFF,0x55,0x12,0xED,0x00,0xFF]
g=[0xFF,0x55,0x40,0xBF,0x00,0xFF]
stopmotion=[0xFF,0x55,0x00,0xFF,0x00,0xFF]

def send(com,action):
    ser=serial.Serial(com,57600)
    ser.write(action)
    ser.write(stopmotion)


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
center_x=None
center_y=None
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
    global center_x
    global center_y
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
        up3 = np.array([100,255,200])
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
        upd2 = np.array([25,200,200])
        lowerd2 = np.array([10,80,100])
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
        
        #=================================顯示畫面=================================#
        #green=ctr
        green=(156,189)
        body=ctr1
        
        face=ctr2
    
        center_x=int((body[0]+face[0])/2)
        center_y=int((body[1]+face[1])/2)
        center=(center_x, center_y)
        target=(36,208)
        cv2.line(img,green,target,(0,255,0),2)#gt
        cv2.line(img,body,face,(0,0,255),2)#pr
        cv2.line(img,center,target,(0,0,255),2)#ct
        cv2.line(img,center,green,(0,255,0),2)#cg
        
        #cv2.circle(img,target,8,(255,255,0),-1)
        
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
    minct_1=0
    maxct_1=0
    minrg=0
    maxrg=0
    minrt=0
    maxrt=0
    mingt_1=0
    maxgt_1=0
    pr=0
    lent=0
    RobotAttack = "COM7"
    print("Set up")
    send(RobotAttack,stopmotion)
    while(true == 1):
        #print('ya')
        if not centroid_x1 is None and not centroid_y1 is None and not centroid_x2 is None and not centroid_y2 is None:
            
            ball = np.array([centroid_x , centroid_y])
            if not ball[0] is None and not ball[1] is None:
                Green = np.array([ball[0] , ball[1]])#球
        
            #Green = np.array([156,189])
            Target = np.array([36,208])#球門
            Red = np.array([centroid_x1 , centroid_y1])#進攻前面
            purple= np.array([centroid_x2 , centroid_y2])#進攻後面
            centerx=int((purple[0]+Red[0])/2)
            centery=int((purple[1]+Red[1])/2)
            Central=np.array([centerx,centery])
            lent=abs(centerx-Green[0])+abs(centery-Green[1])#中心與綠球距離
            distance=Central-Green#直線距離
            distance=int(math.hypot(distance[0], distance[1]))
            #print("Green=>",Green)
            #print("Target=>",Target)
            #print("Red=>",Red)
            #print("purple=>",purple)
            if not Green[0] is  None and not Red[0] is None and not purple[0] is None:
                #centerx=int((purple[0]+Red[0])/2)
                #centery=int((purple[1]+Red[1])/2)
                #Central=np.array([centerx,centery])
                #print(Green)
                #print(Central)
                #lent=abs(centerx-Green[0])+abs(centery-Green[1])
                #distance=Central-Green
                #distance=int(math.hypot(distance[0], distance[1]))
    #=================================定義斜率=================================#
                if float(centerx-Green[0])!=0.0 and float(centery-Green[1])!=0.0:
                    cg=float(centery-Green[1])/float(centerx-Green[0])
                    #print('Central&Green=',cg)
                else:
                    cg=0.0
                if float(centerx-Target[0])!=0.0 and float(centery-Target[1])!=0.0:
                    ct=float(centery-Target[1])/float(centerx-Target[0])
                    #print('Cetral&Target=',ct)
                else:
                    ct=0.0
                if float(Red[1]-Green[1])!=0.0 and float(Red[0] - Green[0])!=0.0:#Front and ball
                    rg=float(Red[1]-Green[1])/float(Red[0] - Green[0])
                else:
                    rg=0.0
                    #print('rg=>',rg)
                if float(purple[1]-Target[1])!=0.0 and float(purple[0] - Target[0])!=0.0:  #Behind and Target
                    pt=float(purple[1]-Target[1])/float(purple[0] - Target[0])
                else:
                    pt=0.0
                    #print('pt=>',pt)
                if float(Red[1]-Target[1])!=0.0 and float(Red[0] - Target[0])!=0.0:   #Front and Target
                    rt=float(Red[1]-Target[1])/float(Red[0] - Target[0])
                else:
                    rt=0.0
                    #print('rt=>',rt)
                if float(purple[1]-Green[1])!=0.0 and float(purple[0] - Green[0])!=0.0:  #Behind and ball
                    pg=float(purple[1]-Green[1])/float(purple[0] - Green[0])
                else:
                    pg=0.0
                    #print('pg=>',pg)
                if float(Green[1]-Target[1])!=0.0 and float(Green[0] - Target[0])!=0.0:
                    gt = float(Green[1]-Target[1])/float(Green[0]-Target[0])
                else:
                    gt=0.0
                    #print("gt=>",gt)
                if (float(purple[1]-Red[1])!=0.0 and float(purple[0]-Red[0])!=0.0):
                    pr=float(purple[1]-Red[1])/float(purple[0]-Red[0])
                else:
                    pr=0.0
                print("pr=>",pr)
                    #n=abs((1/(Red[0] - Green[0])*1.8))#
                    #m=abs(pt-rt)*10#error
                error_ct_1=0.25
                error_gt_1=0.15
                error_3=0.25
                    #print("n:",n)
                    #print("m:",m)
                    #minrg=rg-2*n
                    #maxrg=rg+2*n
                    #print("maxrg=>",maxrg)
                    #print("minrg=>",minrg)
                minct_1=ct-error_ct_1
                maxct_1=ct+error_ct_1
                mingt_1=gt-error_gt_1
                maxgt_1=gt+error_gt_1
                print("cg:",cg)
                print("gt:",gt)
                print("maxct=>",maxct_1)
                print("minct=>",minct_1)
                print("lent:",lent)
                print("maxgt=>",maxgt_1)
                print("mingt=>",mingt_1)
                print("distance=>", distance)
                #mingt_1=gt-error_gt_1
                #maxgt_1=gt+error_gt_1
                mingt_3=gt-0.1
                maxgt_3=gt+0.1
                #print("lent:",lent)
                #print("maxgt=>",maxgt)
                #print("mingt=>",mingt)
                #print("distance=>", distance)
                    #print("lent=>",lent)
        else:
            print("顏色抓取失敗2")
    #====================================進攻邏輯======================================#
        #sleep(0.5)
        if lent>=30:
            if pr<minct_1:
                print("turn right")
                while True:
                    
                    send(RobotAttack,right_s)
                    send(RobotAttack,stopmotion) 
                    #s.write('Cd'.encode())
                    sleep(1)
                    
                    break
            elif pr>maxct_1:
                print("turn left")
                
                while True:
                    send(RobotAttack,left_s)
                    send(RobotAttack,stopmotion) 
                    #s.write('Ca'.encode())
                    sleep(1)
                        
                    break
            elif pr<maxct_1 and pr>minct_1:
                
                if cg>maxgt_1:
                    print("go right")
                    while True:  
                        send(RobotAttack,right)
                        send(RobotAttack,stopmotion) 
                        #.write('Cg'.encode())#往右橫移，會往左邊偏
                        sleep(1)
                    
                    
                        
                        break
                elif cg<mingt_1:
                    print("go left")
                    
                    while True:
                        send(RobotAttack,left)
                        send(RobotAttack,stopmotion) 
                        #s.write('Cf'.encode())
                        sleep(1)
                        
                    
                        break
                elif (cg>mingt_1 and cg<maxgt_1):
                    print("Straight")
                    
                    while True:  
                        send(RobotAttack,forward)
                        send(RobotAttack,stopmotion)
                        #s.write('Cw'.encode())
                        sleep(0.5)
                
                        break
        
        elif lent <30:
            if (pr>minct_1 and pr<maxct_1):
                print("準備踢球")
                print("把機器人移到球的右側")
                '''
                if(cg<-1.3 or cg>=0.0):
                    if(cg<=-1.3):
                        while True:
                            s.write('Cg'.encode())
                            #s.write('Cg'.encode())
                            print("right")
                            sleep(1.5)
                            break
                    elif(cg>=0.0):
                        while True:
                            s.write('Cf'.encode())
                            #s.write('Cg'.encode())
                            print("left")
                            sleep(1.5)
                            break
                '''
                if(cg<0.0):
                     while True:
                         send(RobotAttack,left)
                         send(RobotAttack,stopmotion)
                         #s.write('Cg'.encode())
                         #s.write('Cg'.encode())
                         print("左移")
                         sleep(1)
                         break
                else:
                    if(lent<12):
                        print("右踢")
                        while True:
                            send(RobotAttack,kick)
                            send(RobotAttack,stopmotion)
                            #s.write('Ck'.encode())
                            sleep(1)
                            break
                    else:
                        print("直走")
                        while True:
                            send(RobotAttack,forward)
                            send(RobotAttack,stopmotion)
                            #s.write('Cw'.encode())
                            sleep(0.5)
                            break
            elif pr<minct_1:
                print("右轉")
                while True:  
                    send(RobotAttack,right_s)
                    send(RobotAttack,stopmotion)
                    #s.write('Cd'.encode())
                    sleep(1)
                    break
                
            elif pr>maxct_1:
                print("左轉")
                
                while True:
                    send(RobotAttack,left_s)
                    send(RobotAttack,stopmotion)
                    #s.write('Ca'.encode())
                    sleep(1)
                    
                    break
        
DF2=0 
L_1=0
R_1=0           
mutex = threading.Lock()

def defenseRight():
    global DF2
    global L_1
    global R_1
    RobotDefense1 = "COM5"#gate
    RobotDefense2 = "COM8"#mid
    print("Set up Defense1")
    send(RobotDefense1,stopmotion)

    print("Set up Defense2")
    send(RobotDefense2,stopmotion)


    DF_target_mid = np.array([563 ,247])
    #DF_target_gate = np.array([centroid_xd1 , centroid_y])
    DF_gate_gt= 0
    DF_mid_gt = 0
    DF_mid_cg=0
    DF_gate_cg=0
    uplimit_gate = np.array([596 , 181])
    lowlimit_gate = np.array([602, 313])
    while(True):
        
        if not centroid_xd1 is None and not centroid_yd1 is None and not centroid_xd2 is None and not centroid_yd2 is None:
            Green = np.array([centroid_x , centroid_y])#球
            if not Green[0] is None and not Green[1] is None:
            
                DF_mid = np.array([centroid_xd2 , centroid_yd2])#中場
                DF_gate = np.array([centroid_xd1 , centroid_yd1])#守門員
                DF_target_gate = np.array([centroid_xd1 , centroid_y])
                
                if float(Green[1]-DF_target_mid[1])!=0 and float(Green[0]-DF_target_mid[0])!=0:#for mid
                    DF_mid_gt=float(Green[1]-DF_target_mid[1])/float(Green[0]-DF_target_mid[0])
                if float(Green[1]-DF_target_gate[1])!=0 and float(Green[0]-DF_target_gate[0])!=0:#For gate
                    DF_gate_gt=float(Green[1]-DF_target_gate[1])/float(Green[0]-DF_target_gate[0])
                if float(Green[1]-DF_mid[1])!=0 and float(Green[0]-DF_mid[0])!=0:#for mid
                    DF_mid_cg=float(Green[1]-DF_mid[1])/float(Green[0]-DF_mid[0])
                if float(Green[1]-DF_gate[1])!=0 and float(Green[0]-DF_gate[0])!=0:#for gate
                    DF_gate_cg=float(Green[1]-DF_gate[1])/float(Green[0]-DF_gate[0])
            #===============================================defense2================================
            
                maxDF_mid_gt=DF_mid_gt+0.1
                minDF_mid_gt=DF_mid_gt-0.1
                maxDF_gate_gt=DF_gate_gt+0.1
                minDF_gate_gt=DF_gate_gt-0.1
                if DF_mid_cg>maxDF_mid_gt:
                    print("mid_DefendRight")
                    while True:
                        send(RobotDefense2,right)
                        send(RobotDefense2,stopmotion)
                        #s.write('Ag'.encode())
                        sleep(0.5)
                                
                        break
                if DF_mid_cg<minDF_mid_gt:
                    print("mid_DefendLeft")
                    while True:  
                        send(RobotDefense2,left)
                        send(RobotDefense2,stopmotion)
                        sleep(0.5)
                        send(RobotDefense2,left)
                        send(RobotDefense2,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.5)
                    
                        break
                if DF_gate[1]>lowlimit_gate[1]:
                    print("gate_Defendright")
                    while True:  
                        send(RobotDefense1,right)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.5)
                    
                        break
                    
                elif DF_gate[1]<uplimit_gate[1]:
                    print("gate_DefendLeft")
                    while True:  
                        send(RobotDefense1,left)
                        send(RobotDefense1,stopmotion)
                        sleep(0.5)
                        send(RobotDefense1,left_s)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.5)
                        send(RobotDefense1,left_s)
                        send(RobotDefense1,stopmotion)
                        sleep(0.5)
                        break
                elif DF_gate_cg>maxDF_gate_gt:
                    print("gate_Defendright")
                    while True:  
                        send(RobotDefense1,right)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.5)
                    
                        break
                elif DF_gate_cg<minDF_gate_gt:
                    print("gate_DefendLeft")
                    while True:  
                        send(RobotDefense1,left)
                        send(RobotDefense1,stopmotion)
                        sleep(0.5)
                        send(RobotDefense1,left_s)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.5)
                        send(RobotDefense1,left_s)
                        send(RobotDefense1,stopmotion)
                        sleep(0.5)
                    
                        break
                
                
def defenseleft():
    global DF2
    global L_1
    global R_1
    RobotDefense1 = "COM5"#gate
    RobotDefense2 = "COM8"#mid
    print("Set up Defense1")
    send(RobotDefense1,stopmotion)
    print("Set up Defense2")
    send(RobotDefense2,stopmotion)
    DF_target_mid = np.array([37,260])
    #DF_target_gate = np.array([centroid_xd1 , centroid_y])
    uplimit_gate = np.array([2 ,197 ])
    lowlimit_gate = np.array([5 ,326 ])
    DF_mid_gt = 0
    DF_mid_cg = 0
    DF_gate_gt = 0
    DF_gate_cg = 0
    while(True):
        if not centroid_xd1 is None and not centroid_yd1 is None and not centroid_xd2 is None and not centroid_yd2 is None:
            ball = np.array([centroid_x , centroid_y])
            if not ball[0] is None and not ball[1] is None:
                
                Green = np.array([centroid_x , centroid_y])#球
                DF_mid = np.array([centroid_xd2 , centroid_yd2])#中場
                DF_gate = np.array([centroid_xd1 , centroid_yd1])#守門員
                DF_target_gate = np.array([centroid_xd1 , centroid_y])
                if float(Green[1]-DF_target_mid[1])!=0 and float(Green[0]-DF_target_mid[0])!=0:#for mid
                    DF_mid_gt=float(Green[1]-DF_target_mid[1])/float(Green[0]-DF_target_mid[0])
                if float(Green[1]-DF_target_gate[1])!=0 and float(Green[0]-DF_target_gate[0])!=0:#For gate
                    DF_gate_gt=float(Green[1]-DF_target_gate[1])/float(Green[0]-DF_target_gate[0])
                if float(Green[1]-DF_mid[1])!=0 and float(Green[0]-DF_mid[0])!=0:#for mid
                    DF_mid_cg=float(Green[1]-DF_mid[1])/float(Green[0]-DF_mid[0])
                if float(Green[1]-DF_gate[1])!=0 and float(Green[0]-DF_gate[0])!=0:#for gate
                    DF_gate_cg=float(Green[1]-DF_gate[1])/float(Green[0]-DF_gate[0])
            #===============================================defense2================================
                maxDF_mid_gt=DF_mid_gt+0.1
                minDF_mid_gt=DF_mid_gt-0.1
                maxDF_gate_gt=DF_gate_gt+0.1
                minDF_gate_gt=DF_gate_gt-0.1
                if DF_mid_cg>maxDF_mid_gt:
                    print("mid_DefendRight")
                    while True:
                        send(RobotDefense2,right)
                        send(RobotDefense2,stopmotion)
                        #s.write('Ag'.encode())
                        sleep(0.7)
                        send(RobotDefense2,right_s)
                        send(RobotDefense2,stopmotion)
                        sleep(0.7)
                                
                        break
                if DF_mid_cg<minDF_mid_gt:
                    print("mid_DefendLeft")
                    while True:  
                        send(RobotDefense2,left)
                        send(RobotDefense2,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.7)
                    
                        break
                if DF_gate[1]>lowlimit_gate[1]:
                    print("gate_DefendLeft")
                    while True:  
                        send(RobotDefense1,left)
                        send(RobotDefense1,stopmotion)
                        sleep(0.7)
                        send(RobotDefense1,left_s)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        #s.write('Af'.encode())
                        sleep(0.7)
                    
                        break
                elif DF_gate[1]<uplimit_gate[1]:
                    print("gate_Defendright")
                    while True:  
                        send(RobotDefense1,right)
                        send(RobotDefense1,stopmotion)
       
                        
                        #s.write('Af'.encode())
                        sleep(0.7)
                    
                        break
                elif DF_gate_cg>maxDF_gate_gt:
                    print("gate_Defendright")
                    while True:  
                        send(RobotDefense1,right)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.7)
                    
                        break
                elif DF_gate_cg<minDF_gate_gt:
                    print("gate_DefendLeft")
                    while True:  
                        send(RobotDefense1,left)
                        send(RobotDefense1,stopmotion)
                        sleep(0.7)
                        send(RobotDefense1,left_s)
                        send(RobotDefense1,stopmotion)
                        #s.write('Af'.encode())
                        sleep(0.7)
                    
                        break
                    
def main():
    TestFrame()
    #ShowFrame()
    #Attack1()
    t=threading.Thread(target=ShowFrame) 
    t1 = threading.Thread(target=Attack1)#用法和Process类很相似
    #t3=threading.Thread(target=defense)
    t2 = threading.Thread(target=defenseRight)
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
        

    


