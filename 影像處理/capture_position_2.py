# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 17:42:16 2019

@author: user
"""


import cv2
def OnMouseAction(event,x,y,flags,param):
    global x1,y1
    if(event==cv2.EVENT_LBUTTONDOWN):
        #cv2.circle(frame,(x,y),(0,255,0),-1)
        x1=x
        y1=y
        print(x1,y1)
        

cap=cv2.VideoCapture(1)

PointLow=(301,382)#球門前
PointHigh=(305,99)

#limitLow=(301,382)#球門後
#limitHigh=(305,99)

PointLow1=(445,306)#中線
PointHigh1=(440,181)

x1=0
y1=0
free_kick_ball_1=(66,287)
free_kick_ball_2=(193,236)
free_kick_ball_3=(191,168)
free_kick_target_1=(68,186)#right
free_kick_target_2=(67,216)
free_kick_target_3=(66,275)

a=(470,357)
b=(589,337)
free_kick_ball_4=(472,301)#left_up
free_kick_ball_5=(470,232)#left_mid
free_kick_ball_6=(471,163)#left_down
free_kick_target_4=(602,324)#left
free_kick_target_5=(602,250)
free_kick_target_6=(601,146)
cv2.namedWindow('frame')
cv2.setMouseCallback('frame',OnMouseAction)
while(1):
    ret, image_frame = cap.read()
    #image_frame=cv2.imread("cat002.jpg")
    #cv2.namedWindow('frame')
    #cv2.setMouseCallback('frame',OnMouseAction)#滑鼠移到frame上就會呼叫OnMouseAction
    #cv2.imshow("frame",frame)
    '''
    cv2.circle(image_frame,(x1,y1),4,(255,0,0),-1)
    
    cv2.circle(image_frame,PointLow,8,(255,255,0),-1)
    cv2.circle(image_frame,PointHigh,8,(255,255,0),-1)
    #cv2.circle(image_frame,limitLow,8,(255,255,0),-1)
    #cv2.circle(image_frame,limitHigh,8,(255,255,0),-1)
    cv2.circle(image_frame,PointLow1,8,(255,0,255),-1)
    cv2.circle(image_frame,PointHigh1,8,(255,0,255),-1)
    '''
    #cv2.circle(image_frame,free_kick_target_1,8,(0,255,255),-1)
    #cv2.circle(image_frame,free_kick_target_2,8,(0,255,255),-1)
    #cv2.circle(image_frame,free_kick_target_3,8,(0,255,255),-1)
    #cv2.circle(image_frame,free_kick_ball_4,4,(0,255,255),-1)
    #cv2.circle(image_frame,free_kick_ball_5,4,(0,255,255),-1)
    cv2.circle(image_frame,free_kick_ball_6,4,(0,255,255),-1)
    #cv2.circle(image_frame,free_kick_target_4,4,(0,255,255),-1)
    #cv2.circle(image_frame,free_kick_target_5,4,(0,255,255),-1)
    cv2.circle(image_frame,free_kick_target_6,4,(0,255,255),-1)
    #cv2.circle(image_frame,a,4,(0,255,255),-1)
    #cv2.circle(image_frame,b,4,(0,255,255),-1)
    cv2.imshow("frame",image_frame)
    #cv2.circle(frame,(x,y), 4, (255,0,0),-1)
    if(cv2.waitKey(1)==ord('q')):
        break

#cap1.release()
cv2.destroyAllWindows()