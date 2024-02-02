import rospy 
from sensor_msgs.msg import Image , CameraInfo
from geometry_msgs.msg import TransformStamped,PoseStamped
import cv2 
from cv_bridge import CvBridge
import numpy as np 
import math
import tf
import threading
import time
from demo.srv import robot_move
from demo.msg import pose
from grap import serial_init , grap_init,grap_catch,grap_release
import ctypes 


#===============================================================MODIFY
'''多颜色识别:
1. 替换'det_color(img)'函数的内容为'det_allKinds_color(img)'的内容
2. 替换'debug_image(img)'函数的内容为' debug_allKindsCol_img(img)'的内容
3. 'shift_camera()'中'my_pose.ry = -90 / 1000'表示相机与夹爪的y轴差,向上移动9cm
'''
color_dict={
    'blue':{'hsv_l':np.array([98,63,164]),
            'hsv_h':np.array([157,255,255])},
    'pink':{'hsv_l':np.array([ 155  ,130 ,190]),
            'hsv_h':np.array([177, 150 ,210])},
    'purple':{'hsv_l':np.array([127 , 47,  61]),
            'hsv_h':np.array([132 ,167 ,174])},
    'red':{'hsv_l':np.array([ 170 ,200 ,180]),
            'hsv_h':np.array([179 ,224, 232])},
    'yellow':{'hsv_l':np.array([26 ,158 ,223]),
            'hsv_h':np.array([28 ,212 ,237])},
    'green':{'hsv_l':np.array([ 49 , 65 ,137]),
           'hsv_h':np.array([70, 255 ,175])}, 
    'orange':{'hsv_l':np.array([ 8 ,150 ,174]),
            'hsv_h':np.array([25 ,215, 240])}, 
}

{# hsv检测结果
# purple:Average HSV: [129.3661401  133.592176   119.65536349]
# HSV Range (Min): [127  47  61]
# HSV Range (Max): [132 167 174]
# yellow:Average HSV: [ 27.53199883 201.38961134 228.89165693]
# HSV Range (Min): [ 26 158 223]
# HSV Range (Max): [ 28 212 237]
# red：Average HSV: [173.50425971 208.30295598 191.58054731]
# HSV Range (Min): [  0  59 137]
# HSV Range (Max): [179 224 232]
# pink:Average HSV: [163.66702716 140.51964672 201.65068493]
# HSV Range (Min): [  0   5 153]
# HSV Range (Max): [177 162 255]
# orirenge:Average HSV: [  9.81439661 182.76767819 228.22717008]
# HSV Range (Min): [  8  12 173]
# HSV Range (Max): [ 26 215 245]
}

color_list = []
hsv_mask_list = []

for color, values in color_dict.items():
    color_list.append(color)
    hsv_mask_list.append([values['hsv_l'], values['hsv_h']])

def det_allKinds_color(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    color_masks = {} 

    for color_name, color_mask in zip(color_list, hsv_mask_list):
        mask_l, mask_h = color_mask
        print(mask_l,mask_h)
        mask = cv2.inRange(hsv_image, mask_l, mask_h)
        # print(f"Detected Mask for {color_name}:", mask,cv2.countNonZero(mask))
        color_masks[color_name] =  mask 

    best_match_color_index = max(range(len(color_masks)), key=lambda k: cv2.countNonZero(color_masks[color_list[k]]))
    best_match_color = color_list[best_match_color_index]
    print(best_match_color, cv2.countNonZero(color_masks[best_match_color]))
    best_match_mask_l, best_match_mask_h = hsv_mask_list[best_match_color_index]

    mask = color_masks[best_match_color]
    # cv2.imwrite(f'result_mask_{best_match_color}.png', mask)
    res = det_rect(mask)
    # print(type(res))
    if res :
        return best_match_mask_l,best_match_mask_h
    else:
        return None
    
def debug_allKindsCol_img(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    color_masks = {} 

    for color_name, color_mask in zip(color_list, hsv_mask_list):
        mask_l, mask_h = color_mask
        print(mask_l,mask_h)
        mask = cv2.inRange(hsv_image, mask_l, mask_h)
        # print(f"Detected Mask for {color_name}:", mask,cv2.countNonZero(mask))
        color_masks[color_name] =  mask 

    best_match_color_index = max(range(len(color_masks)), key=lambda k: cv2.countNonZero(color_masks[color_list[k]]))
    best_match_color = color_list[best_match_color_index]
    print(best_match_color, cv2.countNonZero(color_masks[best_match_color]))
    # best_match_mask_l, best_match_mask_h = hsv_mask_list[best_match_color_index]

    mask = color_masks[best_match_color]
    # cv2.imwrite(f'result_mask_{best_match_color}.png', mask)
    res = det_rect(mask)
    # print(type(res))
    if res:
        cv2.drawContours(img,[res[0]],0,(0,0,255),2)
    cv2.imshow('rect',img)
    cv2.waitKey(1)  
#===============================================================addColor



demo_lib = None
robot_service = None
ser = None

color_det = False

# 调整物块颜色
blue_mask_l = np.array([97,112,98])
blue_mask_h = np.array([142,255,255])
# blue_mask_l = np.array([0,66,165])
# blue_mask_h = np.array([255,255,255])
det_mask_l = None
det_mask_h = None

run_done = False

def arm_grap_catch():
    time.sleep(0.5)
    grap_catch(ser)
    time.sleep(1.5)

def arm_grap_release():
    time.sleep(0.5)
    grap_release(ser)
    time.sleep(1.5)

def wait_task(task_id,dest_name):
    #下发任务
    task_id = int(task_id)
    demo_lib.woosh_robot_run_task_group.argtypes = [ctypes.c_int]
    #任务组id  需要修改
    task_id = ctypes.c_int(task_id)
    res = demo_lib.woosh_robot_run_task_group(task_id)
    print(res)


    demo_lib.woosh_robot_listen.argtypes = [ctypes.c_char_p,ctypes.c_char_p]
    #目标点名称 需要修改
    dest_name = dest_name
    dest_name = dest_name.encode()
    dest_name = ctypes.c_char_p(dest_name)

    # 监听是否到到等待状态  可修改
    dest_state = 'ActionWait'
    dest_state = dest_state.encode()
    dest_state = ctypes.c_char_p(dest_state)

    #监听目标点 123  状态wait.  当底盘状态为 在目标点123等待的时候返回 --- 阻塞
    demo_lib.woosh_robot_listen(dest_name , dest_state)



def wait_done():
    global run_done
    run_done = False
    while True:
        if run_done:
            return

def det_rect(mask):
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask , cv2.MORPH_OPEN,kernel)
    mask = cv2.morphologyEx(mask , cv2.MORPH_CLOSE,kernel)

    contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    rect = None
    box = 0 
    cx = 0 
    cy = 0

    
    for cnt in contours:
        (x,y,w,h) = cv2.boundingRect(cnt)
        if w < 50 and h < 50:
            continue
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        cx = np.mean(box[:,0])
        cy = np.mean(box[:,1])
        box = np.int0(box)
        break

        #cv2.drawContours(cv_image,[box],0,(0,0,255),2)
    if rect :
        #box_angle = rect[2]
        #diff_x = cx - cv_image.shape[1]/2
        #diff_y = cy - cv_image.shape[0]/2
        return [box,cx,cy,rect[2]]
    else:
        return None

def debug_img(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, blue_mask_l, blue_mask_h)   
    #img = cv2.rotate(img, cv2.ROTATE_180)
    res = det_rect(mask)
    if res:
        cv2.drawContours(img,[res[0]],0,(0,0,255),2)
    # flipped_img = cv2.flip(img, 0)
    # cv2.imshow('rect',flipped_img)
    cv2.imshow('rect',img)
    cv2.waitKey(1)  
    # hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # color_masks = {} 

    # for color_name, color_mask in zip(color_list, hsv_mask_list):
    #     mask_l, mask_h = color_mask
    #     print(mask_l,mask_h)
    #     mask = cv2.inRange(hsv_image, mask_l, mask_h)
    #     # print(f"Detected Mask for {color_name}:", mask,cv2.countNonZero(mask))
    #     color_masks[color_name] =  mask 

    # best_match_color_index = max(range(len(color_masks)), key=lambda k: cv2.countNonZero(color_masks[color_list[k]]))
    # best_match_color = color_list[best_match_color_index]
    # print(best_match_color, cv2.countNonZero(color_masks[best_match_color]))
    # # best_match_mask_l, best_match_mask_h = hsv_mask_list[best_match_color_index]

    # mask = color_masks[best_match_color]
    # # cv2.imwrite(f'result_mask_{best_match_color}.png', mask)
    # res = det_rect(mask)
    # # print(type(res))
    # if res:
    #     cv2.drawContours(img,[res[0]],0,(0,0,255),2)
    # cv2.imshow('rect',img)
    # cv2.waitKey(1)  


def det_color(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, blue_mask_l, blue_mask_h)

    res = det_rect(mask)

    if res :
        return blue_mask_l,blue_mask_h
    else:
        return None
    

def shift_camera_3_1():
    time.sleep(0.5)
    my_pose = pose()
    my_pose.ry = -100 / 1000
    my_pose.rx = 0
    my_pose.y = 3
    if robot_service :
        res = robot_service(my_pose,10.0,True)
        print(res)
        #print('ok')
    else:
        exit(0)

def shift_camera_1_1():
    time.sleep(0.5)
    my_pose = pose()
    my_pose.ry = -90 / 1000
    my_pose.rx = 0
    my_pose.y = 3
    if robot_service :
        res = robot_service(my_pose,10.0,True)
        print("in shift_camera_1_1:res",res)
        #print('ok')
    else:
        exit(0)

def go_back_home():
    print("HOME")
    run_point(-181.639,47.643,572.853,-88.358,1.226,-135.359,50.0) # 1-1工件 3-1工件

def go_back_home2():
    print("HOME")
    run_point(-128.349,-6.319,1027.885,-88.737,1.11,-130.874,50.0) # 1-1工件 3-1工件

def send_box_example_1_1():
    print("send_box")
    time.sleep(1)
    run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)
    run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    time.sleep(1)
    # run_point(-138.629,357.543,140.000,-179.764,-0.222,45.291,10.0)
    # time.sleep(1)

def send_box_example_3_1():
    print("send_box")
    time.sleep(1)
    run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)
    run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    time.sleep(1)
    run_point(-174.378,420.125,140.000,160.045,2.924,44.804,10.0)
    time.sleep(1)

def send_box_release_on():
    print("send_realease")
    time.sleep(1)
    run_point(-153.666,362.749,360.198,-179.764,-0.222,45.291,50.0)
    time.sleep(1)

def shift_arm(pose,speed):
    if robot_service :
        res = robot_service(pose,speed,True)
        print('in shift_arm:res',res)
        #print('ok')
    else:
        exit(0)    


def run_point(x,y,z,r,p,_y_,speed,shift=False):
    point = pose()
    point.rx = x/1000
    point.ry = y/1000
    point.rz = z/1000
    point.r = r
    point.p = p
    point.y = _y_
    #print(robot_service)
    if robot_service:
        res = robot_service(point,speed , False)
        return res 


  

def move_to_center(img):
    global det_mask_l ,det_mask_h , color_det,run_done
    if not color_det:
        print('start to det color(in move_to_center )')
        res = det_color(img)
        if res:
            #print('test ok')
            det_mask_l = res[0]
            det_mask_h = res[1]
        else:
            print('color det failed')
        color_det = True
    if  not run_done:
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # add mask 
        mask = cv2.inRange(hsv_image, det_mask_l, det_mask_h)   

        res = det_rect(mask)
        if res:
            cx = res[1]
            cy = res[2]
            diff_x = cx - img.shape[1]/2
            diff_y = cy - img.shape[0]/2
            my_pose = pose()
            my_pose.rx = -diff_x /1000
            my_pose.ry = -diff_y / 1000
            if robot_service :
                res = robot_service(my_pose,10.0,True)
                # time.sleep(0.5)
                print(res)
                run_done = True
                #print('ok')
            else:
                exit(0)




def rotate_angle(img):
    #blue  [88,66,157]  [147,255,255]
    global det_mask_l ,det_mask_h , color_det,run_done
    if not color_det:
        print('start to det color(in rotate_angle )')
        res = det_color(img)
        if res:
            #print('test ok')
            det_mask_l = res[0]
            det_mask_h = res[1]
        else:
            print('color det failed')
        color_det = True
    if  not run_done:
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # add mask
        mask = cv2.inRange(hsv_image, det_mask_l, det_mask_h)   

        res = det_rect(mask)
    
        if res:
            cv2.drawContours(img,[res[0]],0,(0,0,255),2)
            cv2.imshow('rect',img)
            cv2.waitKey(1)

            box_angle = res[3]
            if abs(box_angle) < 45 :
            # + 
                rotate_angle = -abs(box_angle)
            else:
                rotate_angle = (90.0 -abs(box_angle))
    
            my_pose = pose()
            my_pose.y = rotate_angle
    
            if robot_service :
                res = robot_service(my_pose,10.0,True)
                print('rotate angle')
            #print(res)
                if res:
                    run_done = True
                else:
                    print('robot move error')
            else:
                exit(0)


def all_init():
    global demo_lib , robot_service , ser
    try:
        # #step 1 : connect woosh robot
        # demo_lib = ctypes.CDLL('librobot_demo.so')
        # demo_lib.woosh_robot_init.argtypes = [ctypes.c_char_p]
        # #ip 地址需要修改
        # ip_addr = '192.168.1.226'
        # ip_addr = ip_addr.encode()
        # c_ip_addr = ctypes.c_char_p(ip_addr)

        # #连接底盘
        # demo_lib.woosh_robot_init(c_ip_addr)

        #step 2 : connect grap 
        ser = serial_init()
        #wait init done.
        grap_init(ser)
        time.sleep(3)
        grap_release(ser)
        time.sleep(1)
        print("here1")
        #step 3 : connect camera ros
        rospy.init_node('color_box',anonymous=True)
        print("here2")
        rospy.wait_for_service('move_arm',timeout = 2.0)

        #step 4 : go to home point.
        service = rospy.ServiceProxy('/move_arm',robot_move)
        print("here3")

        robot_service = service
        if service is None:
            print('service error')
            exit(0)

        home_pose = pose()
        home_pose.rx = -0.318476
        home_pose.ry = 0.302017
        home_pose.rz = 0.248979
        home_pose.r = -179.613
        home_pose.p = -2.263
        home_pose.y = 44.034
        print('goto start point')
        #res = service(home_pose,40.0,False)
        #if not res:
        #    print('go home error')
        

    except:
        print('utils error')
        exit(0)



