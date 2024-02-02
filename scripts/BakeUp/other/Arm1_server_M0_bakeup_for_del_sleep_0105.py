import utils
# from utils import *
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

avg_depth = None

image_cb = None
lock = None
###########################MODIFY
request_lock = threading.Lock()
###########################MODIFY
show_img = None
thread = None
TEST=0
# TEST=1
RUNNING=not(TEST)

def update_image_cb_to_none():
    global image_cb
    image_cb=None

def cb(color_image):
    global image_cb,show_img,lock
    #print(color_image.width , color_image.height)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(color_image,desired_encoding='passthrough')
    cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)

    utils.debug_img(cv_image)
    
    if lock :
        if(lock.acquire(blocking=False)):
            # lock.acquire()
            try:
                if image_cb:
                    image_cb(cv_image)
            except Exception as e:
                print('cb error',str(e))         
            finally:
                lock.release()


def depth_cb(img):
    global avg_depth
    #print(img.width,img.height)
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
    #print('from cb',avg_depth)


def listen_msg():
    rospy.spin()

def log(info):
    with open("a.txt","a") as f:
        f.write(info)

#===============================================================grab
#===============================================================grab
def grab_arm1_1_1_go():
    pass
def grab_arm1_1_1_back(pre_point):
    #first to camera point
    print(pre_point)
    print('grab_arm1_1_1_back here1')
    utils.run_point(492.987,-486.749,730.369,-87.166,1.779,-136.289,30.0) 
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2]+40,pre_point[3],pre_point[4],pre_point[5],pre_point[6])
    print('grab_arm1_1_1_back here2')
    time.sleep(1)

def grab_arm1_2_1_go():
    time.sleep(1)
    print('in grab_arm1_2_1_go')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    # utils.run_point(220.579,-331.908,715.624,-131.899,-1.424,-134.045,50.0) ##liuxin-2023.12.09.20.28
    time.sleep(1)

def grab_arm1_2_1_back(pre_point):
    #first to camera point
    print(pre_point)
    time.sleep(1)
    print('in grab_arm1_2_1_back')
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],20)
    time.sleep(1)
    print('in grab_arm1_2_1_back inflection_2')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0)
    # utils.run_point(220.579,-331.908,715.624,-131.899,-1.424,-134.045,50.0)##liuxin-2023.12.09.20.28
    print('end grab_arm1_2_1_back inflection_2')
    time.sleep(1)
  
def grab_arm1_3_1_go():
    # 11.30 10:00 with Yang to define
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    ### maybe pre grab point
    time.sleep(1)
    # utils.run_point(-41.237,-145.638,-118.262,53.439,92.993,1.179,50.0) ##lx_2.23.12.09.20.16
    #  utils.run_point(292.358,-354.835,61.116,-123.169,-1.433,-137.049,50.0) # 3-1工件

def grab_arm1_3_1_back(pre_point):
    # 11.30 10:00 with Yang to define
    print("in grab_arm1_3_1_back")
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],20)
    print("in grab_arm1_3_1_back_mid")
    time.sleep(1)
    # utils.run_point(-41.237,-145.638,-118.262,53.439,92.993,1.179,50.0)
    time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    print("in grab_arm1_3_1_back done")


def grab_arm1_robot01_go():
    # time.sleep(1)
    # utils.run_point(-280.844,-4.359,957.809,-89.715,0.197,19.012,50.0)
    # time.sleep(1)
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot01_back(pre_point):
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

def grab_arm1_robot02_go():
    print("grab_arm1_robot02_go start")
    time.sleep(1)
    utils.run_point(-131.616,352.766,368.439,-178.767,-0.336,39.426,50.0)
    time.sleep(1)
    print("grab_arm1_robot02_go end")

    # utils.run_point(-331.425,-1.974,755.266,-172.486,0.688,85.688,50.0)
    # time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot02_back(pre_point):
    # sure
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)
    time.sleep(1)
    # utils.run_point(-331.425,-1.974,755.266,-172.486,0.688,85.688,50.0)
    # time.sleep(1)
    #utils.run_point(-131.616,352.766,368.439,-178.767,-0.336,39.426,50.0)
    #utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)


def grab_arm1_robot03_go():
    time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot03_back(pre_point):
    #to be sure......
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)
    time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    time.sleep(1)

    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

def grab_arm1_robot04_go():
    pass
    time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot04_back(pre_point):
    #to be sure......

    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)
    time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)
    time.sleep(1)

    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

def default():
    print('default')
    utils.go_back_home2()
    print('default:already ai home')


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
    print('in loosen_arm1_2_1_go')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    print('end loosen_arm1_2_1_go')

def loosen_arm1_2_1_back():
    print('in loosen_arm1_2_1_back')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    print('end loosen_arm1_2_1_back')
    

def loosen_arm1_3_1_go():
    print('in loosen_arm1_3_1_go')
    time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    time.sleep(1)

def loosen_arm1_3_1_back():    
    print('in loosen_arm1_3_1_back')
    time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    time.sleep(1)

def loosen_arm1_robot01_go():
    print("send_box")
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)
    utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    time.sleep(1)
    
def loosen_arm1_robot01_back():
   utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
   time.sleep(1)
   utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
   time.sleep(1)

def loosen_arm1_robot02_go():
    # not sure about where to send,temp set robot02
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)
    utils.run_point(-326.509,153.445,366.456,176.756,-4.161,47.011,30.0)
    time.sleep(1)
    # utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    # time.sleep(1)
    # ## may be target loosen point
    # ###utils.run_point(-174.378,420.125,140.000,160.045,2.924,44.804,10.0)
    # time.sleep(1)
    
def loosen_arm1_robot02_back():
    # not sure about where to send,temp set robot02
    time.sleep(1)
    utils.run_point(-326.509,153.445,366.456,176.756,-4.161,47.011,30.0)
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)
    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    # time.sleep(1)

def loosen_arm1_robot03_go():
    # not sure about where to send,temp set robot03
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)
    utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    time.sleep(1)
    ## may be target loosen point
    ###utils.run_point(-174.378,420.125,140.000,160.045,2.924,44.804,10.0)
    time.sleep(1)
    
def loosen_arm1_robot03_back():
    # not sure about where to send,temp set robot02
    time.sleep(1)
    utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

def loosen_arm1_robot04_go():
    # not sure about where to send,temp set robot04
    time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)##lx2023.12.17-16.07
    time.sleep(1)
    utils.run_point(-491.788,284.279,308.577,179.192,-3.057,45.628,50.0)
    time.sleep(1)

def loosen_arm1_robot04_back():
    # not sure about where to send,temp set robot04
    time.sleep(1)
    utils.run_point(-491.788,284.279,308.577,179.192,-3.057,45.628,50.0)
    time.sleep(1)
    utils.run_point(-339.945,347.493,560.013,-179.319,0.146,41.171,50.0)##lx2023.12.17-16.07
    time.sleep(1)

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
    print("default+++++++")


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

app = Flask(__name__)
@app.route('/grab', methods=['POST','GET'])
def grab():
    global request_loc
    if not request_lock.acquire(blocking=False):
        return jsonify({'multi_requests_error': 'Another grab is in progressing. Please try again later.'}), 400
    try:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>in grab server>>>>>>>>>")
        global image_cb,show_img,lock,avg_depth,thread
        image_cb = None
        data = request.get_json()
        print(data)
        print(data['action'])
        print(data['param']['position'])
        if 'action' in data and 'param' in data :
            action_class=data['action']
            action_value=data['param']['position']
            # action_value='arm1-1-1'
            param_value = data['param']
            print("###################",action_value,param_value)
            # TEST=data['test']
            # RUNNING=not(TEST)
            # print("########################################################################")
            is_robot=1
            is_arm1_1=0
            if "arm1-" in action_value :
                is_robot=0
                if "arm1-1-" in action_value :
                    is_arm1_1=1
            print("########################################################################123456789")
            print(is_robot)
            print(is_arm1_1)
            utils.update_dict(is_robot,is_arm1_1)
            # utils.wangwei_hello()
            print("########################################################################")
            info=f"receive grab command:\n{action_value}\n{param_value}\n"
            log(info)

        if RUNNING:
            print('start arm_move service for grab')
            lock = threading.Lock()
            #########################
            rospy.Subscriber( '/camera_2/color/image_raw', Image,cb,queue_size = 1)
            rospy.Subscriber( '/camera_2/depth/image_raw', Image,depth_cb)
            #########################
            thread = threading.Thread(target = listen_msg)
            thread.start()
            time.sleep(2.0)
            print('demo start')

            print("going to standby point......")
            utils.go_back_home2()  
            print('end go home')
            utils.arm_grap_release()

            #=============================================================infection point
            print("========grab go inflection points......")
            grab_go_switch_case(action_value)
            #=============================================================infection point
            
            print("going to grab ready point from rmf......")
            grab_pre_point=[np.float32(param_value["x"]),
                        np.float32(param_value["y"]),
                        np.float32(param_value["z"]),
                        np.float32(param_value["rx"]),
                        np.float32(param_value["ry"]),
                        np.float32(param_value["rz"]),
                        np.float32(param_value["speed"])]
            
            utils.run_point(np.float32(param_value["x"]),
                            np.float32(param_value["y"]),
                            np.float32(param_value["z"]),
                            np.float32(param_value["rx"]),
                            np.float32(param_value["ry"]),
                            np.float32(param_value["rz"]),
                            np.float32(param_value["speed"]))
            print('already arrive at start point !!!!!! ')
            time.sleep(6)
            print('adjusting grab rotate...... ')
            lock.acquire()
            image_cb = utils.rotate_angle
            lock.release()
            print('waiting1...... ')
            utils.wait_done()
            time.sleep(1)
            time.sleep(0.1)

            print('Adjusting camera  to center...... ')
            print('1st to center...... ')
            lock.acquire()
            image_cb = utils.move_to_center
            lock.release()
            utils.wait_done()
            time.sleep(1)

            print('2st to center...... ')
            lock.acquire()
            image_cb = utils.move_to_center
            lock.release()

            utils.wait_done()
            # time.sleep(3.0)

            time.sleep(1)

            if(action_value=="arm1-2-1" or action_value=="arm1-2-2" or action_value=="arm1-2-3" or action_value=="arm1-2-4" or action_value=="arm1-2-5"):
                print('3st to center for arm1-2-1...5...... ')
                lock.acquire()
                image_cb = utils.move_to_center
                lock.release()
                utils.wait_done()
                # time.sleep(3.0)
                time.sleep(1)
                
            time.sleep(5.0)


            obj_depth = avg_depth
            print('obj_depth=',obj_depth)

            #shift to camera
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
                print(f"********** obj_depth = {obj_depth} is not in range! **********")
                if is_robot:
                    my_pose.rz = 0.235
                else:
                    my_pose.rz = 0.25
            
            if is_robot:
                if not 0.23 <= my_pose.rz <= 0.245:
                    print(f"********** rz = {my_pose.rz} is not in range! **********")
                    my_pose.rz = 0.245 if my_pose.rz > 0.245 else 0.23
    
            print("%s$$$$$$$$$$$$$$" %(my_pose.rz) )
            utils.shift_arm(my_pose,10.0)

            utils.arm_grap_catch()

            #=============================================================infection point
            print("========grab back inflection points......")
            grab_back_switch_case(action_value,pre_point=grab_pre_point)
            #=============================================================infection point

            
            
            print("going to standby point......")
            utils.go_back_home2()
            print("already at standby point......")
            print('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<grab DONE ^^<<<<<<<<<<<<<<')

            # rospy.signal_shutdown('demo running done')
            # thread.join()

        if TEST:
            print("test grab successfully!")
            # time.sleep(4) 
        return jsonify({'action':f'{action_class}',
                        'position': f'{action_value}'})
    finally:
        request_lock.release()

@app.route('/loosen', methods=['POST','GET'])
def loosen():
    if not request_lock.acquire(blocking=False):
        return jsonify({'multi_requests_error': 'Another grab is in progressing. Please try again later.'}), 400
    try:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>in grab server>>>>>>>>>")
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
            print("========loosen go inflection points......")
            loosen_go_switch_case(action_value)
            #=============================================================infection point
            

            print("going to pre_loosen point......")
            utils.run_point(np.float32(param_value["x"]),
                            np.float32(param_value["y"]),
                            np.float32(param_value["z"]),
                            np.float32(param_value["rx"]),
                            np.float32(param_value["ry"]),
                            np.float32(param_value["rz"]),
                            np.float32(param_value["speed"]))
            time.sleep(1)
            utils.arm_grap_release()

            #=============================================================infection point
            print("========loosen back inflection points......")
            loosen_back_switch_case(action_value)
            #=============================================================infection point

            # utils.send_box_release_on()
            print('already drop it')

            print("going to standby point......")
            utils.go_back_home2()
            print("already at standby point......")
            print('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<loosen DONE ^^<<<<<<<<')
            time.sleep(2)
            # rospy.signal_shutdown('loosen ok')
        if TEST:
            print("test loosen successfully!")

        return jsonify({'action':f'{action_class}',
                        'position': f'{action_value}'})
    finally:
        request_lock.release()


if __name__ == '__main__':
    utils.all_init()
    app.run(host='192.168.1.226', port=7070)

