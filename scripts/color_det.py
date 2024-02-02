import rospy 
from sensor_msgs.msg import Image , CameraInfo
import cv2 
from cv_bridge import CvBridge
import numpy as np 
import math
import time
import utils

def nothing(x):
    pass




def cb(color_image):

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(color_image,desired_encoding='passthrough')
    cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    '''
    #################
    # 
    blue_mask_l = np.array([ 98,63,182])
    blue_mask_h = np.array([157,255,255])
    mask = cv2.inRange(hsv, blue_mask_l, blue_mask_h)
    res = utils.det_rect(mask)
    if res:
        cv2.drawContours(cv_image,[res[0]],0,(0,0,255),2)
        cv2.imshow('rect',cv_image)
        cv2.waitKey(1)  
    ###########################
    '''


    cv_image=cv2.flip(cv_image,-1)
    cv2.imshow("src", cv_image)
    

    cv2.namedWindow("img2", cv2.WINDOW_NORMAL)
    cv2.namedWindow('img2',1)
    cv2.createTrackbar('Hlow','img2',96,180,nothing)
    cv2.createTrackbar('Hup','img2',116,180,nothing)
    cv2.createTrackbar('Slow','img2',36,255,nothing)
    cv2.createTrackbar('Sup','img2',255,255,nothing)
    cv2.createTrackbar('Vlow','img2',81,255,nothing)
    cv2.createTrackbar('Vup','img2',255,255,nothing)



    while True:
        hlow = cv2.getTrackbarPos('Hlow', 'img2')
        hup = cv2.getTrackbarPos('Hup', 'img2')
        slow = cv2.getTrackbarPos('Slow', 'img2')
        sup = cv2.getTrackbarPos('Sup', 'img2')
        vlow = cv2.getTrackbarPos('Vlow', 'img2')
        vup = cv2.getTrackbarPos('Vup', 'img2')
        lower_red = np.array([hlow, slow, vlow])
        upper_red = np.array([hup, sup, vup])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        #    img2 = cv2.bitwise_and(img, img, mask=mask)
    
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask , cv2.MORPH_OPEN,kernel)
        mask = cv2.morphologyEx(mask , cv2.MORPH_CLOSE,kernel)
        img2 = mask 
        img2=cv2.flip(img2,-1)
        original_size=img2.shape[:2]
        # print(img2.shape[:2])
        new_size = (int(original_size[1] * 0.7), int(original_size[0] * 0.7))

        img2=cv2.resize(img2, new_size)
        cv2.imshow("img2", img2)
        cv2.waitKey(1)


rospy.init_node('abc',anonymous=True)
rospy.Subscriber( '/camera_2/color/image_raw', Image,cb)
rospy.spin()

