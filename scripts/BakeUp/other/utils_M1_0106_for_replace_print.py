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

# import Arm1_server 
from Arm1_server_M1_for_log import update_image_cb_to_none,grab_back_switch_case,get_action_value,get_grab_pre_point
#===============================================================MODIFY

import datetime
color_dict = {}
color_list = []
hsv_mask_list = []

col_dict_daylight_robot = {
    # "square": {"hsv_l": np.array([0, 76, 96]), "hsv_h": np.array([180, 255, 255])},
    # "red": {"hsv_l": np.array([0, 53, 76]), "hsv_h": np.array([10, 255, 230])},
    # "red": {"hsv_l": np.array([0, 90, 97]), "hsv_h": np.array([180, 255, 255])}, ###wangwei 12.20
    # "red": {"hsv_l": np.array([0, 157, 94]), "hsv_h": np.array([180, 255, 183])}, ###LiuXin2023.12.17-13.36
    # "red": {"hsv_l": np.array([0, 80, 0]), "hsv_h": np.array([33, 255, 255])}, ###LiuXin2023.12.17-13.36
    # "blue": {"hsv_l": np.array([97,112,98]), "hsv_h": np.array([142,255,255])}, ###wangwei 12.20
    "blue": {"hsv_l": np.array([99,80,115]), "hsv_h": np.array([115,222,209])}, ###wangwei 12.20
    # "pink": {"hsv_l": np.array([83, 50, 111]), "hsv_h": np.array([180, 220, 230])}, ###wangwei 12.20
    "pink": {"hsv_l": np.array([159, 71, 124]), "hsv_h": np.array([180, 145, 200])}, ###wangwei 12.20
    # "yellow": {"hsv_l": np.array([13, 82, 223]), "hsv_h": np.array([35, 255, 255])},
    "green": {"hsv_l": np.array([30, 65, 100]), "hsv_h": np.array([44, 150, 200])},
    # "orange": {"hsv_l": np.array([2, 82, 183]), s"hsv_h": np.array([15, 255, 255])},
    # "purple": {"hsv_l": np.array([115, 62, 108]), "hsv_h": np.array([142, 255, 255])},
    # "pale blue": {"hsv_l": np.array([78, 24, 207]), "hsv_h": np.array([120, 255, 255])},
    # "peony red": {"hsv_l": np.array([150, 60, 175]), "hsv_h": np.array([187, 255, 255])},
    # "light red": {"hsv_l": np.array([157, 72, 183]), "hsv_h": np.array([187, 255, 255])},
}
col_dict_daylight_shelf_1 = {
    "blue": {"hsv_l": np.array([100,81,141]), "hsv_h": np.array([115,220,255])},
    "pink": {"hsv_l": np.array([130, 90, 74]), "hsv_h": np.array([180, 140, 255])},
    "green": {"hsv_l": np.array([30, 70, 65]), "hsv_h": np.array([60, 135, 220])},
}
col_dict_daylight_shelf_23 = {
    "blue": {"hsv_l": np.array([99,80,115]), "hsv_h": np.array([115,222,209])},
    "pink": {"hsv_l": np.array([160, 71, 74]), "hsv_h": np.array([180, 145, 200])},
    "green": {"hsv_l": np.array([30, 66, 80]), "hsv_h": np.array([44, 103, 150])},
}
col_dict_night_robot = {
    "blue": {"hsv_l": np.array([99,80,115]), "hsv_h": np.array([115,222,209])},
    "pink": {"hsv_l": np.array([159, 71, 124]), "hsv_h": np.array([180, 145, 200])},
    "green": {"hsv_l": np.array([30, 65, 100]), "hsv_h": np.array([44, 150, 200])},
}
col_dict_night_shelf_1 = {
    "blue": {"hsv_l": np.array([99,80,115]), "hsv_h": np.array([115,240,230])},
    "pink": {"hsv_l": np.array([159, 90, 124]), "hsv_h": np.array([180, 150, 230])},
    "green": {"hsv_l": np.array([30, 64, 100]), "hsv_h": np.array([60, 150, 230])},
}
col_dict_night_shelf_23 = {
    "blue": {"hsv_l": np.array([99,89,115]), "hsv_h": np.array([115,230,209])},
    "pink": {"hsv_l": np.array([160, 71, 124]), "hsv_h": np.array([180, 145, 200])},
    "green": {"hsv_l": np.array([30, 64, 100]), "hsv_h": np.array([45, 150, 200])},
}

def get_current_time():
    return datetime.datetime.now().time()

def update_dict(is_robot,is_arm1_1):
    global color_dict,hsv_mask_list
    hsv_mask_list=[]
    current_time = get_current_time()
    print("current_time",current_time)
    
    # 8:00-18:00->daytime
    daylight_start = datetime.time(8, 0)
    daylight_end = datetime.time(18, 0)
    
    if daylight_start <= current_time <= daylight_end:
        if is_robot:
            color_dict= col_dict_daylight_robot
        else:
            if is_arm1_1:
                color_dict= col_dict_daylight_shelf_1
            else:
                color_dict= col_dict_daylight_shelf_23   
    else:
        if is_robot:
            color_dict= col_dict_night_robot
        else:
            if is_arm1_1:
                color_dict= col_dict_night_shelf_1
            else:
                color_dict= col_dict_night_shelf_23
    print(color_dict)
    for color, values in color_dict.items():
        color_list.append(color)
        hsv_mask_list.append([values['hsv_l'], values['hsv_h']])
    for hsv_mask_list_value in hsv_mask_list:
        print(hsv_mask_list_value)




#===============================================================addColor

demo_lib = None
robot_service = None
ser = None

color_det = False
max_try_cnt=10


# blue_mask_l = np.array([97,112,98])
# blue_mask_h = np.array([142,255,255])
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


def rotate_angle(img):
    global det_mask_l ,det_mask_h , color_det,run_done,max_try_cnt
    max_try_cnt=max_try_cnt-1
    if(max_try_cnt>=0):
        print("max_try_cnt",max_try_cnt)
        if not color_det:
            print('start to det color(in rotate_angle )')
            res = det_color(img)
            if res:
                #print('test ok')
                det_mask_l = res[0]
                det_mask_h = res[1]
                color_det = True
            else:
                print('color det failed')
                color_det = False
                return None
        if  not run_done:
            print("rotate->wait for wait_done sleep 1s")
            # time.sleep(0.2)
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # add mask
            found_color = None  
            found_masks = {} 
            mask_l=None
            mask_h=None
            size_threshold = img.shape[0] * img.shape[1] * 0.04
            for color_name, hsv_range in zip(color_list, hsv_mask_list):
                mask_l, mask_h = hsv_range
                mask = cv2.inRange(hsv_image, mask_l, mask_h)
                mask_nonzero=np.count_nonzero(mask==255) 
                print("before judge mask_nonzero")
                if mask_nonzero > size_threshold:
                    found_color = color_name 
                    found_masks[found_color] = mask  
                    print("size_threshold=",size_threshold)
                    print(f"{color_name}_mask_nonzero=",mask_nonzero)
                    break  
            if found_color is not None:  
                print("Found color:", found_color)      
                res = det_rect(found_masks[found_color]) 
                if res:
                    # cv2.drawContours(img,[res[0]],0,(0,0,255),2)
                    # cv2.imshow('rect',img)
                    # cv2.waitKey(1)
                    box_angle = res[3]
                    if abs(box_angle) < 45 :
                        rotate_angle = -abs(box_angle)
                    else:
                        rotate_angle = (90.0 -abs(box_angle))
                    my_pose = pose()
                    my_pose.y = rotate_angle
                    if robot_service :
                        res = robot_service(my_pose,10.0,True)
                        print('rotate angle,res=',res)
                        if res:
                            run_done = True
                        else:
                            print('robot move error')
                    else:
                        exit(0)
    else:
        grab_back_switch_case(get_action_value(),get_grab_pre_point())
        go_back_home2()
        exit(0)

def move_to_center(img):
    # print("just in to_center sleep 1s")
    # time.sleep(1)
    global det_mask_l ,det_mask_h , color_det,run_done
    if not color_det:
        print('start to det color(in move_to_center )')
        res = det_color(img)
        if res:
            #print('test ok')
            det_mask_l = res[0]
            det_mask_h = res[1]
            color_det = True
        else:
            print('color det failed')
            color_det = False
            return None
    if  not run_done:
        print("to_center->wait for wait_done sleep 1s")
        # time.sleep(0.2)
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        found_color = None  
        found_masks = {} 
        mask_l=None
        mask_h=None
        size_threshold = img.shape[0] * img.shape[1] * 0.04
        for color_name, hsv_range in zip(color_list, hsv_mask_list):
            mask_l, mask_h = hsv_range
            mask = cv2.inRange(hsv_image, mask_l, mask_h)
            mask_nonzero=np.count_nonzero(mask==255) 
            print("to_center->before judge mask_nonzero")
            if mask_nonzero > size_threshold:
                found_color = color_name 
                found_masks[found_color] = mask  
                print("size_threshold=",size_threshold)
                print(f"{color_name}_mask_nonzero=",mask_nonzero)
                break  
        if found_color is not None:  
            print("Found color:", found_color)      
            res = det_rect(found_masks[found_color])  
            if res :
                print("detect success")
                cx = res[1]
                cy = res[2]
                diff_x = cx - img.shape[1]/2
                diff_y = cy - img.shape[0]/2
                my_pose = pose()
                my_pose.rx = -diff_x /1000
                my_pose.ry = -diff_y / 1000
                print("my_pose.ry=",my_pose.ry," ","my_pose.rx=",my_pose.rx)
                if robot_service :
                    res = robot_service(my_pose,10.0,True)
                    print("move_to_center:res->",res)
                    run_done = True
                else:
                    exit(0)
            else:
                print("detect failure")

def wait_task(task_id,dest_name):
    #�·�����
    task_id = int(task_id)
    demo_lib.woosh_robot_run_task_group.argtypes = [ctypes.c_int]
    #������id  ��Ҫ�޸�
    task_id = ctypes.c_int(task_id)
    res = demo_lib.woosh_robot_run_task_group(task_id)
    print(res)


    demo_lib.woosh_robot_listen.argtypes = [ctypes.c_char_p,ctypes.c_char_p]
    #Ŀ������� ��Ҫ�޸�
    dest_name = dest_name
    dest_name = dest_name.encode()
    dest_name = ctypes.c_char_p(dest_name)

    # �����Ƿ񵽵��ȴ�״̬  ���޸�
    dest_state = 'ActionWait'
    dest_state = dest_state.encode()
    dest_state = ctypes.c_char_p(dest_state)

    #����Ŀ��� 123  ״̬wait.  ������״̬Ϊ ��Ŀ���123�ȴ���ʱ�򷵻� --- ����
    demo_lib.woosh_robot_listen(dest_name , dest_state)



def wait_done():
    global run_done,color_det,max_try_cnt
    run_done = False
    while True:
        if run_done:
            update_image_cb_to_none()
            max_try_cnt=10
            color_det=False
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
    cnt = max(contours, key=cv2.contourArea)
    (x,y,w,h) = cv2.boundingRect(cnt)
    if w < 35 and h < 35:
        return None
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    cx = np.mean(box[:,0])
    cy = np.mean(box[:,1])
    box = np.int0(box)
    if rect :
        return [box,cx,cy,rect[2]]
    else:
        return None

def debug_img(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    found_color = None  
    found_masks = {} 
    mask_l=None
    mask_h=None
    size_threshold = img.shape[0] * img.shape[1] * 0.04
    for color_name, hsv_range in zip(color_list, hsv_mask_list):
        mask_l, mask_h = hsv_range
        mask = cv2.inRange(hsv_image, mask_l, mask_h)
        mask_nonzero=np.count_nonzero(mask==255) 
        # print("{color_name}_mask_nonzero=",mask_nonzero)
        # print("size_threshold=",size_threshold)
        if mask_nonzero > size_threshold:
            found_color = color_name 
            found_masks[found_color] = mask  
            break  
    if found_color is not None:  
        # print("Found color:", found_color)
        res = det_rect(found_masks[found_color])  
        if res:
            cv2.drawContours(img, [res[0]], 0, (0, 0, 255), 2)
    cv2.imshow('rect', img)
    # cv2.imshow('mask', found_masks[found_color])
    cv2.waitKey(1)  
    # else:
    #     print("Func->debug_img:No color found")


       
    

def det_color(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    found_color = None  
    found_masks = {} 
    mask_l=None
    mask_h=None
    size_threshold = img.shape[0] * img.shape[1] * 0.04
    print("in det color",hsv_mask_list)
    for color_name, hsv_range in zip(color_list, hsv_mask_list):
        mask_l, mask_h = hsv_range
        mask = cv2.inRange(hsv_image, mask_l, mask_h)
        mask_nonzero=np.count_nonzero(mask==255) 
        print("in det color")
        if mask_nonzero > size_threshold:
            found_color = color_name 
            found_masks[found_color] = mask  
            # print("size_threshold=",size_threshold)
            # print("{color_name}_mask_nonzero=",mask_nonzero)
            break  
    if found_color is not None:  
        print("in det color Found color:", found_color)      
        res = det_rect(found_masks[found_color])  
        if res :
            return mask_l,mask_h
        else:
            return None
    # else:
    #     print("Func->det_img:No color found")



def shift_camera_3_1():
    time.sleep(0.5)
    my_pose = pose()
    my_pose.ry = -100 / 1000
    my_pose.rx = 0
    my_pose.y = 3
    if robot_service :
        res = robot_service(my_pose,10.0,True)
        print("in shift_camera_2_1:res",res)
        #print('ok')
    else:
        exit(0)

def shift_camera_2_1():
    time.sleep(0.5)
    my_pose = pose()
    my_pose.ry = -65 / 1000
    my_pose.rx = 0
    my_pose.y = 3
    if robot_service :
        res = robot_service(my_pose,10.0,True)
        print("in shift_camera_2_1:res",res)
        #print('ok')
    else:
        exit(0)

def shift_camera_1_1():
    time.sleep(0.5)
    my_pose = pose()
    my_pose.ry = -85 / 1000
    my_pose.rx = 0
    my_pose.y = 3
    if robot_service :
        res = robot_service(my_pose,10.0,True)
        print("in shift_camera_1_1:res",res)
        #print('ok')
    else:
        exit(0)

def shift_camera_robot04():
    time.sleep(0.5)
    my_pose = pose()
    my_pose.ry = -65 / 1000
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
    run_point(-181.639,47.643,572.853,-88.358,1.226,-135.359,50.0) # 1-1���� 3-1����

def go_back_home2():
    print("HOME")
    run_point(-128.349,-6.319,1027.885,-88.737,1.11,-130.874,50.0) # 1-1���� 3-1����
    # run_point(1.798,-144.924,1075.885,-88.737,1.11,-134.874,50.0)##lx-2023.12.17-15.46

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


  


def all_init():
    global demo_lib , robot_service , ser
    try:
        # #step 1 : connect woosh robot
        # demo_lib = ctypes.CDLL('librobot_demo.so')
        # demo_lib.woosh_robot_init.argtypes = [ctypes.c_char_p]
        # #ip ��ַ��Ҫ�޸�
        # ip_addr = '192.168.1.226'
        # ip_addr = ip_addr.encode()
        # c_ip_addr = ctypes.c_char_p(ip_addr)

        # #���ӵ���
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


# if __name__ =="__main__":
#     det_color = det_color()
#     det_color()

#     pass



