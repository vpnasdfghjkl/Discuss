import utils
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

from flask import Flask, request,jsonify
import datetime
import os

avg_depth = None

image_cb = None
lock = None
# ###########################MODIFY
# request_lock = threading.Lock()
# ###########################MODIFY
show_img = None
thread = None
TEST=0
# TEST=1
RUNNING=not(TEST)

# def update_image_cb_to_none():
#     global image_cb,lock
#     Log_Arm1_server_print("in update_image_cb_to_none")
#     while(1):    
#         Log_Arm1_server_print("before acquire lock")
#         if(lock.acquire()):
#             Log_Arm1_server_print("already get lock")
#             image_cb=None
#             break
#     lock.release()
#     Log_Arm1_server_print("finish update_image_cb_to_none,image_cb=",image_cb)


# def real_time_img(color_image):
#     bridge = CvBridge()
#     cv_image = bridge.imgmsg_to_cv2(color_image,desired_encoding='passthrough')
#     cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)
#     cv2.imshow("Real-time img",cv_image)
    
def cb(color_image):
    global image_cb,show_img,lock
    #Log_Arm1_server_print(color_image.width , color_image.height)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(color_image,desired_encoding='passthrough')
    cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)

    utils.debug_img(cv_image)
    if lock :
        if(lock.acquire()):
            try:
                # Log_Arm1_server_print("image_cb==None?",image_cb==None)
                if image_cb:
                    Log_Arm1_server_print("before")
                    image_cb(cv_image)
                    Log_Arm1_server_print("after")
            except Exception as e:
                Log_Arm1_server_print('cb error',str(e))         
            finally:
                lock.release()


def depth_cb(img):
    global avg_depth
    #Log_Arm1_server_print(img.width,img.height)
    # get center index
    shift_x = img.width  /2
    shift_y = img.height /2


    index = img.width * shift_y + shift_x
    data1 = (img.data[int(index) * 2]  ) + (img.data[int(index) * 2 +1] << 8)
    shift_y = shift_y + 1
    index = img.width * shift_y + shift_x
    data2 = (img.data[int(index) * 2]  ) + (img.data[int(index) * 2 +1] << 8)
    shift_x = shift_x + 1
    index = img.width * shift_y + shift_x
    data3 = (img.data[int(index) * 2]  ) + (img.data[int(index) * 2 +1] << 8)
    shift_y = shift_y -1
    data4 = (img.data[int(index) * 2]  ) + (img.data[int(index) * 2 +1] << 8)
    # mm  convert -> m
    avg_depth = (data1 + data2 + data3 + data4) /4
    avg_depth = avg_depth * 0.001
    #Log_Arm1_server_print('from cb',avg_depth)


def listen_msg():
    rospy.spin()

def log(info):
    with open("a.txt","a") as f:
        f.write(info)

def get_current_time():
    return datetime.datetime.now().time()

def Log_Arm1_server_start0():
    current_dir = os.path.dirname(os.path.abspath(__file__))  
    log_file_path = os.path.join(current_dir, "Log_Arm1_server.txt")  
    with open(log_file_path, "a") as f:
        f.write("\n\n\n\n")
        current_time = get_current_time()
        str_for_write="|||||||||||||||||||||||||in grab Current_time:"+str(current_time)
        f.write(str_for_write)
        f.write("\n")
        
def Log_Arm1_server_start(*args):
    current_dir = os.path.dirname(os.path.abspath(__file__))  
    log_file_path = os.path.join(current_dir, "Log_Arm1_server.txt")  
    with open(log_file_path, "a") as f:
        info = ''.join(map(str, args))
        print(info)
        current_time = get_current_time()
        str_for_write = "Current_time:" + str(current_time) + "\n" + info
        f.write(str_for_write)
        f.write("\n")
    
def Log_Arm1_server_print(*args):
    info=''.join(map(str,args))
    print(info)
    current_dir = os.path.dirname(os.path.abspath(__file__))  
    log_file_path = os.path.join(current_dir, "Log_Arm1_server.txt")  
    with open(log_file_path, "a") as f:
        f.write(info)
        f.write("\n")
#===============================================================grab
#===============================================================grab
def grab_arm1_1_1_go():
    pass
def grab_arm1_1_1_back(pre_point):
    #first to camera point
    Log_Arm1_server_print(pre_point)
    Log_Arm1_server_print('grab_arm1_1_1_back here1')
    utils.run_point(492.987,-486.749,730.369,-87.166,1.779,-136.289,100.0) 
    #time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2]+40,pre_point[3],pre_point[4],pre_point[5],pre_point[6])
    Log_Arm1_server_print('grab_arm1_1_1_back here2')
    #time.sleep(1)

def grab_arm1_2_1_go():
    #time.sleep(1)
    Log_Arm1_server_print('in grab_arm1_2_1_go')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    # utils.run_point(220.579,-331.908,715.624,-131.899,-1.424,-134.045,50.0) ##liuxin-2023.12.09.20.28
    #time.sleep(1)

def grab_arm1_2_1_back(pre_point):
    #first to camera point
    Log_Arm1_server_print(pre_point)
    #time.sleep(1)
    Log_Arm1_server_print('in grab_arm1_2_1_back')
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],20)
    #time.sleep(1)
    Log_Arm1_server_print('in grab_arm1_2_1_back inflection_2')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0)
    # utils.run_point(220.579,-331.908,715.624,-131.899,-1.424,-134.045,50.0)##liuxin-2023.12.09.20.28
    Log_Arm1_server_print('end grab_arm1_2_1_back inflection_2')
    #time.sleep(1)
  
def grab_arm1_3_1_go():
    # 11.30 10:00 with Yang to define
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    ### maybe pre grab point
    #time.sleep(1)
    # utils.run_point(-41.237,-145.638,-118.262,53.439,92.993,1.179,50.0) ##lx_2.23.12.09.20.16
    # utils.run_point(292.358,-354.835,61.116,-123.169,-1.433,-137.049,50.0) # 3-1工件

def grab_arm1_3_1_back(pre_point):
    # 11.30 10:00 with Yang to define
    Log_Arm1_server_print("in grab_arm1_3_1_back")
    #time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],20)
    Log_Arm1_server_print("in grab_arm1_3_1_back_mid")
    #time.sleep(1)
    # utils.run_point(-41.237,-145.638,-118.262,53.439,92.993,1.179,50.0)
    #time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    Log_Arm1_server_print("in grab_arm1_3_1_back done")


def grab_arm1_robot01_go():
    # #time.sleep(1)
    # utils.run_point(-280.844,-4.359,957.809,-89.715,0.197,19.012,50.0)
    # #time.sleep(1)
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,100.0)
    #time.sleep(1)

    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # #time.sleep(1)

def grab_arm1_robot01_back(pre_point):
    #time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],100)
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,100.0)
    #time.sleep(1)

def grab_arm1_robot02_go():
    Log_Arm1_server_print("grab_arm1_robot02_go start")
    #time.sleep(1)
    utils.run_point(-131.616,352.766,368.439,-178.767,-0.336,39.426,50.0)
    #time.sleep(1)
    Log_Arm1_server_print("grab_arm1_robot02_go end")

    # utils.run_point(-331.425,-1.974,755.266,-172.486,0.688,85.688,50.0)
    # #time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # #time.sleep(1)

def grab_arm1_robot02_back(pre_point):
    # sure
    #time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],50)
    #time.sleep(1)
    # utils.run_point(-331.425,-1.974,755.266,-172.486,0.688,85.688,50.0)
    # #time.sleep(1)
    #utils.run_point(-131.616,352.766,368.439,-178.767,-0.336,39.426,50.0)
    #utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)


def grab_arm1_robot03_go():
    #time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    #time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # #time.sleep(1)

def grab_arm1_robot03_back(pre_point):
    #to be sure......
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],50)
    #time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    #time.sleep(1)

    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    #time.sleep(1)

def grab_arm1_robot04_go():
    pass
    #time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    #time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # #time.sleep(1)

def grab_arm1_robot04_back(pre_point):
    #to be sure......

    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],50)
    #time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    #time.sleep(1)

    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    #time.sleep(1)

def default():
    Log_Arm1_server_print('default')
    utils.go_back_home2()
    Log_Arm1_server_print('default:already ai home')


grab_go_options = {
    'arm1-1-1': grab_arm1_1_1_go,
    'arm1-1-2': grab_arm1_1_1_go,
    'arm1-1-3': grab_arm1_1_1_go,
    'arm1-1-4': grab_arm1_1_1_go,
    'arm1-1-5': grab_arm1_1_1_go,

    'arm1-2-1':grab_arm1_2_1_go,
    'arm1-2-2':grab_arm1_2_1_go,
    'arm1-2-3':grab_arm1_2_1_go,
    'arm1-2-4':grab_arm1_2_1_go,
    'arm1-2-5':grab_arm1_2_1_go,

    'arm1-3-1':grab_arm1_3_1_go,
    'arm1-3-2':grab_arm1_3_1_go,
    'arm1-3-3':grab_arm1_3_1_go,
    'arm1-3-4':grab_arm1_3_1_go,
    'arm1-3-5':grab_arm1_3_1_go,

    'arm1robot01': grab_arm1_robot01_go,
    'arm1robot02': grab_arm1_robot02_go,
    'arm1robot03': grab_arm1_robot03_go,
    'arm1robot04': grab_arm1_robot04_go,
}
grab_back_options = {
    'arm1-1-1': grab_arm1_1_1_back,
    'arm1-1-2': grab_arm1_1_1_back,
    'arm1-1-3': grab_arm1_1_1_back,
    'arm1-1-4': grab_arm1_1_1_back,
    'arm1-1-5': grab_arm1_1_1_back,

    'arm1-2-1':grab_arm1_2_1_back,
    'arm1-2-2':grab_arm1_2_1_back,
    'arm1-2-3':grab_arm1_2_1_back,
    'arm1-2-4':grab_arm1_2_1_back,
    'arm1-2-5':grab_arm1_2_1_back,
    
    'arm1-3-1':grab_arm1_3_1_back,
    'arm1-3-2':grab_arm1_3_1_back,
    'arm1-3-3':grab_arm1_3_1_back,
    'arm1-3-4':grab_arm1_3_1_back,
    'arm1-3-5':grab_arm1_3_1_back,

    'arm1robot01': grab_arm1_robot01_back,
    'arm1robot02': grab_arm1_robot02_back,
    'arm1robot03': grab_arm1_robot03_back,
    'arm1robot04': grab_arm1_robot04_back,
}

def grab_go_switch_case(action_value):
    func = grab_go_options.get(action_value, default)
    return func()

def grab_back_switch_case(action_value,pre_point):
    func = grab_back_options.get(action_value, default)
    return func(pre_point)

#=================================================================================loosen
#=================================================================================loosen
def loosen_arm1_1_1_go():
   #to be sure......
    pass
    
def loosen_arm1_1_1_back():
   #to be sure......
    pass

def loosen_arm1_2_1_go():
    Log_Arm1_server_print('in loosen_arm1_2_1_go')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    Log_Arm1_server_print('end loosen_arm1_2_1_go')

def loosen_arm1_2_1_back():
    Log_Arm1_server_print('in loosen_arm1_2_1_back')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    Log_Arm1_server_print('end loosen_arm1_2_1_back')
    

def loosen_arm1_3_1_go():
    Log_Arm1_server_print('in loosen_arm1_3_1_go')
    #time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    #time.sleep(1)

def loosen_arm1_3_1_back():    
    Log_Arm1_server_print('in loosen_arm1_3_1_back')
    #time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    #time.sleep(1)

def loosen_arm1_robot01_go():
    Log_Arm1_server_print("send_box")
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,100.0)
    #time.sleep(1)
    utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,100.0)
    #time.sleep(1)
    
def loosen_arm1_robot01_back():
   utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,100.0)
   #time.sleep(1)
   utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,100.0)
   #time.sleep(1)

def loosen_arm1_robot02_go():
    # not sure about where to send,temp set robot02
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    #time.sleep(1)
    utils.run_point(-326.509,153.445,366.456,176.756,-4.161,47.011,50.0)
    #time.sleep(1)
    # utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    # #time.sleep(1)
    # ## may be target loosen point
    # ###utils.run_point(-174.378,420.125,140.000,160.045,2.924,44.804,10.0)
    # #time.sleep(1)
    
def loosen_arm1_robot02_back():
    # not sure about where to send,temp set robot02
    #time.sleep(1)
    utils.run_point(-326.509,153.445,366.456,176.756,-4.161,47.011,50.0)
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    #time.sleep(1)
    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    # #time.sleep(1)

def loosen_arm1_robot03_go():
    # not sure about where to send,temp set robot03
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    #time.sleep(1)
    utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    #time.sleep(1)
    ## may be target loosen point
    ###utils.run_point(-174.378,420.125,140.000,160.045,2.924,44.804,10.0)
    #time.sleep(1)
    
def loosen_arm1_robot03_back():
    # not sure about where to send,temp set robot02
    #time.sleep(1)
    utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    #time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    #time.sleep(1)

def loosen_arm1_robot04_go():
    # not sure about where to send,temp set robot04
    #time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)##lx2023.12.17-16.07
    #time.sleep(1)
    utils.run_point(-491.788,284.279,308.577,179.192,-3.057,45.628,50.0)
    #time.sleep(1)

def loosen_arm1_robot04_back():
    # not sure about where to send,temp set robot04
    #time.sleep(1)
    utils.run_point(-491.788,284.279,308.577,179.192,-3.057,45.628,50.0)
    #time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)##lx2023.12.17-16.07
    #time.sleep(1)

def default():
    pass


loosen_go_options = {
    'arm1-1-1': loosen_arm1_1_1_go,
    'arm1-1-2': loosen_arm1_1_1_go,
    'arm1-1-3': loosen_arm1_1_1_go,
    'arm1-1-4': loosen_arm1_1_1_go,
    'arm1-1-5': loosen_arm1_1_1_go,
    
    'arm1-2-1':loosen_arm1_2_1_go,
    'arm1-2-2':loosen_arm1_2_1_go,
    'arm1-2-3':loosen_arm1_2_1_go,
    'arm1-2-4':loosen_arm1_2_1_go,
    'arm1-2-5':loosen_arm1_2_1_go,

    'arm1-3-1':loosen_arm1_3_1_go,
    'arm1-3-2':loosen_arm1_3_1_go,
    'arm1-3-3':loosen_arm1_3_1_go,
    'arm1-3-4':loosen_arm1_3_1_go,
    'arm1-3-5':loosen_arm1_3_1_go,

    'arm1robot01': loosen_arm1_robot01_go,
    'arm1robot02': loosen_arm1_robot02_go,
    'arm1robot03': loosen_arm1_robot03_go,
    'arm1robot04': loosen_arm1_robot04_go,
    
}


loosen_back_options = {
    'arm1-1-1': loosen_arm1_1_1_back,
    'arm1-1-2': loosen_arm1_1_1_back,
    'arm1-1-3': loosen_arm1_1_1_back,
    'arm1-1-4': loosen_arm1_1_1_back,
    'arm1-1-5': loosen_arm1_1_1_back,

    'arm1-2-1':loosen_arm1_2_1_back,
    'arm1-2-2':loosen_arm1_2_1_back,
    'arm1-2-3':loosen_arm1_2_1_back,
    'arm1-2-4':loosen_arm1_2_1_back,
    'arm1-2-5':loosen_arm1_2_1_back,

    'arm1-3-1':loosen_arm1_3_1_back,
    'arm1-3-2':loosen_arm1_3_1_back,
    'arm1-3-3':loosen_arm1_3_1_back,
    'arm1-3-4':loosen_arm1_3_1_back,
    'arm1-3-5':loosen_arm1_3_1_back,

    'arm1robot01': loosen_arm1_robot01_back,
    'arm1robot02': loosen_arm1_robot02_back,
    'arm1robot03': loosen_arm1_robot03_back,
    'arm1robot04': loosen_arm1_robot04_back,
}
def loosen_go_switch_case(action_value):
    func = loosen_go_options.get(action_value, default)
    return func()

def loosen_back_switch_case(action_value):
    func = loosen_back_options.get(action_value, default)
    return func()



#=================================================================================shift_camera
#=================================================================================shift_camera
def default_shift_camera():
    utils.shift_camera_1_1
    Log_Arm1_server_print("default+++++++")


# shift_camera_options = {
#     'arm1robot04': utils.shift_camera_robot04,
#     'arm1-2-1': utils.shift_camera_2_1,
#     'arm1-2-2': utils.shift_camera_2_1,
#     'arm1-2-3': utils.shift_camera_2_1,
#     'arm1-2-4': utils.shift_camera_2_1,
#     'arm1-2-5': utils.shift_camera_2_1,
# }

# def shift_camera_switch_case(action_value):
#     func = shift_camera_options.get(action_value, default_shift_camera)
#     return func()
    
grab_pre_point=None
action_value=None
def get_grab_pre_point():
    global grab_pre_point
    return grab_pre_point
def get_action_value():
    global action_value
    return action_value

lock = threading.Lock()
app = Flask(__name__)
@app.route('/grab', methods=['POST','GET'])
def grab():
    global action_value,grab_pre_point,request_lock
    try:
        Log_Arm1_server_start0()
        Log_Arm1_server_print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>in grab server>>>>>>>>>")
        global image_cb,show_img,lock,avg_depth,thread
        image_cb = None
        data = request.get_json()
        Log_Arm1_server_print(data)
        Log_Arm1_server_print(data['action'])
        Log_Arm1_server_print(data['param']['position'])
        if 'action' in data and 'param' in data :
            action_class=data['action']
            action_value=data['param']['position']
            # action_value='arm1-1-1'
            param_value = data['param']
            Log_Arm1_server_print("###################",action_value,param_value)
            # TEST=data['test']
            # RUNNING=not(TEST)
            # Log_Arm1_server_print("########################################################################")
            is_robot=1
            is_arm1_1=0
            if "arm1-" in action_value :
                is_robot=0
                if "arm1-1-" in action_value :
                    is_arm1_1=1
            Log_Arm1_server_print("########################################################################123456789")
            Log_Arm1_server_print(is_robot)
            Log_Arm1_server_print(is_arm1_1)
            utils.update_dict(is_robot,is_arm1_1)
            Log_Arm1_server_print("########################################################################")
            info=f"receive command:{action_value}:{param_value}\n"
            Log_Arm1_server_start(info)

        if RUNNING:
            Log_Arm1_server_print('Start arm_move service for grab')
            
            #########################S
            rospy.Subscriber( '/camera_2/color/image_raw', Image,cb,queue_size = 1)
            # rospy.Subscriber( '/camera_2/color/image_raw', Image,real_time_img,queue_size = 1)
            # rospy.Subscriber( '/camera_2/color/image_raw', Image,cb)
            rospy.Subscriber( '/camera_2/depth/image_raw', Image,depth_cb)
            #########################
            thread = threading.Thread(target = listen_msg)
            thread.start()

            # Log_Arm1_server_print("Run to catch cube for fast test......")
            # time.sleep(3)
            
            Log_Arm1_server_print("Going to standby point......")
            utils.go_back_home2()  
            utils.arm_grap_release()
            rotate_cnt=2
            while(rotate_cnt>0):
                utils.rotate_ag=False
                utils.reach_max_try_cnt=False
                rotate_cnt=rotate_cnt-1
                Log_Arm1_server_print("rotate_cnt=",rotate_cnt)
                #=============================================================infection point
                Log_Arm1_server_print("========Grab go inflection points......")
                grab_go_switch_case(action_value)
                #=============================================================infection point
                
                param_value["speed"]=100.0
                grab_pre_point=[np.float32(param_value["x"]),np.float32(param_value["y"]),np.float32(param_value["z"]),np.float32(param_value["rx"]),np.float32(param_value["ry"]),np.float32(param_value["rz"]),np.float32(param_value["speed"]),]
                
                utils.run_point(np.float32(param_value["x"]),np.float32(param_value["y"]),np.float32(param_value["z"]),np.float32(param_value["rx"]),np.float32(param_value["ry"]),np.float32(param_value["rz"]),np.float32(param_value["speed"]),)
                Log_Arm1_server_print('Already arrive at start point')
                time.sleep(1)

                Log_Arm1_server_print('Rotating...... ')
                lock.acquire()
                image_cb = utils.rotate_angle
                lock.release()
                Log_Arm1_server_print('Waiting1...... ')
                utils.wait_done()
                lock.acquire()
                image_cb = None
                lock.release()

                if not utils.rotate_ag:
                    Log_Arm1_server_print("Rotated!!!!!!")
                    break
                elif(rotate_cnt>0):
                    grab_back_switch_case(action_value,pre_point=grab_pre_point)
                    utils.go_back_home2()
                    Log_Arm1_server_print("go home for 2 nd recognize")
                    lock.acquire()
                    image_cb = None
                    lock.release()
            
            if not utils.reach_max_try_cnt:
                Log_Arm1_server_print("utils.reach_max_try_cnt:",utils.reach_max_try_cnt)
                Log_Arm1_server_print('Adjusting camera  to center...... ')
                Log_Arm1_server_print('1st to center...... ')
                lock.acquire()
                image_cb = utils.move_to_center
                lock.release()
                utils.wait_done()
                lock.acquire()
                image_cb = None
                lock.release()
                time.sleep(0.5)

                Log_Arm1_server_print('2st to center...... ')
                lock.acquire()
                image_cb = utils.move_to_center
                lock.release()

                utils.wait_done()
                lock.acquire()
                image_cb = None
                lock.release()

                time.sleep(0.5)

                # if(action_value=="arm1-2-1" or action_value=="arm1-2-2" or action_value=="arm1-2-3" or action_value=="arm1-2-4" or action_value=="arm1-2-5"):
                #     Log_Arm1_server_print('3st to center for arm1-2-1...5...... ')
                #     lock.acquire()
                #     image_cb = utils.move_to_center
                #     lock.release()
                #     utils.wait_done()
                    
                #     lock.acquire()
                #     image_cb = None
                #     lock.release()
                #     # time.sleep(3.0)
                #     time.sleep(1)
                    
                # time.sleep(1.0)

                obj_depth = avg_depth
                Log_Arm1_server_print('obj_depth=',obj_depth)

                #shift to camera
                if("robot" in action_value):
                    Log_Arm1_server_print(" if(robot in action_value):")
                    utils.shift_camera_robot04()
                else:
                    Log_Arm1_server_print(" if(arm1- in action_value):")
                    utils.shift_camera_1_1() # 1-1
                # utils.shift_camera_3_1() # 3-1
                #=============================================================shift camera
                # shift_camera_switch_case(action_value)
                #=============================================================shift camera


                time.sleep(0.1)
            
                my_pose = pose()
                if obj_depth > 0 :   
                    my_pose.rz = obj_depth - 0.10
                else:
                    Log_Arm1_server_print(f"********** obj_depth = {obj_depth} is not in range! **********")
                    if is_robot:
                        my_pose.rz = 0.235
                    else:
                        my_pose.rz = 0.25
                
                if is_robot:
                    if not 0.23 <= my_pose.rz <= 0.245:
                        Log_Arm1_server_print(f"********** rz = {my_pose.rz} is not in range! **********")
                        my_pose.rz = 0.245 if my_pose.rz > 0.245 else 0.23
        
                Log_Arm1_server_print("%s$$$$$$$$$$$$$$" %(my_pose.rz) )
                utils.shift_arm(my_pose,80.0)

                utils.arm_grap_catch()

                #=============================================================infection point
                Log_Arm1_server_print("========grab back inflection points......")
                grab_back_switch_case(action_value,pre_point=grab_pre_point)
                #=============================================================infection point

                
                
                Log_Arm1_server_print("going to standby point......")
                utils.go_back_home2()
                Log_Arm1_server_print("already at standby point......")
                Log_Arm1_server_print('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<grab DONE ^^<<<<<<<<<<<<<<')

                # rospy.signal_shutdown('demo running done')
                # thread.join()
                lock.acquire()
                image_cb = None
                lock.release()
            else:
                grab_back_switch_case(action_value,pre_point=grab_pre_point)
                utils.go_back_home2()
                Log_Arm1_server_print("reach_max_try_cnt grab next......")
                
                lock.acquire()
                image_cb = None
                lock.release()
            # rospy.signal_shutdown('demo running done')
            # thread.join()

        if TEST:
            Log_Arm1_server_print("test grab successfully!")
            # time.sleep(4) 
        return jsonify({'action':f'{action_class}',
                        'position': f'{action_value}'})
    finally:
        # request_lock.release()
        Log_Arm1_server_print("request try ok")

@app.route('/loosen', methods=['POST','GET'])
def loosen():
    # if not request_lock.acquire(blocking=False):
    #     return jsonify({'multi_requests_error': 'Another grab is in progressing. Please try again later.'}), 400
    try:
        Log_Arm1_server_print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>in loosen server>>>>>>>>>")
        global image_cb,show_img,lock,avg_depth,thread
        data = request.get_json()
        if 'action' in data and 'param' in data :
            action_class=data['action']
            action_value=data['param']['position']
            param_value = data['param']
            # TEST=data['test']
            # RUNNING=not(TEST)
            info=f"loosen:\n{action_value}\n{param_value}\n"
            log(info)

        if RUNNING:
            #=============================================================infection point
            Log_Arm1_server_print("========loosen go inflection points......")
            loosen_go_switch_case(action_value)
            #=============================================================infection point
            
            Log_Arm1_server_print("going to pre_loosen point......")
            param_value["speed"]=100.0
            utils.run_point(np.float32(param_value["x"]),
                            np.float32(param_value["y"]),
                            np.float32(param_value["z"]),
                            np.float32(param_value["rx"]),
                            np.float32(param_value["ry"]),
                            np.float32(param_value["rz"]),
                            np.float32(param_value["speed"]),
                            )
            #time.sleep(1)
            utils.arm_grap_release()

            #=============================================================infection point
            Log_Arm1_server_print("========loosen back inflection points......")
            loosen_back_switch_case(action_value)
            #=============================================================infection point

            # utils.send_box_release_on()
            Log_Arm1_server_print('already drop it')

            Log_Arm1_server_print("going to standby point......")
            utils.go_back_home2()
            Log_Arm1_server_print("already at standby point......")
            Log_Arm1_server_print('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<loosen DONE ^^<<<<<<<<')
            # time.sleep(2)
            # rospy.signal_shutdown('loosen ok')
        if TEST:
            Log_Arm1_server_print("test loosen successfully!")

        return jsonify({'action':f'{action_class}',
                        'position': f'{action_value}'})
    finally:
        Log_Arm1_server_print("request loosen try ok")
        # request_lock.release()


if __name__ == '__main__':
    utils.all_init()
    app.run(host='192.168.1.226', port=7070)

