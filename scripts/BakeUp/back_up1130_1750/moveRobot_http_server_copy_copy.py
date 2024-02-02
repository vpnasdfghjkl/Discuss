import utils as utils
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

show_img = None
thread = None
TEST=1
RUNNING=not(TEST)

def cb(color_image):
    global image_cb,show_img,lock
    #print(color_image.width , color_image.height)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(color_image,desired_encoding='passthrough')
    cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)

    utils.debug_img(cv_image)

    if lock :
        lock.acquire()
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

#=========================================grab
def grab_arm1_1_1_go():
    pass
def grab_arm1_1_1_back(pre_point):
    #first to camera point
    print(pre_point)
    print('here1')
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2]+40,pre_point[3],pre_point[4],pre_point[5],pre_point[6])
    print('here2')
    time.sleep(1)


def grab_arm1_2_1_go():
    time.sleep(1)
    print('in grab_arm1_2_1_go')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    time.sleep(1)

    

def grab_arm1_2_1_back(pre_point):
    #first to camera point
    print(pre_point)
    time.sleep(1)
    print('in grab_arm1_2_1_back')
    utils.run_point(pre_point[0],pre_point[1],pre_point[2]+80,pre_point[3],pre_point[4],pre_point[5],20)
    time.sleep(1)
    print('in grab_arm1_2_1_back inflection_2')
    utils.run_point(268.964,-302.565,500.265,-94.269,0.099,-134.459,50.0) 
    print('end grab_arm1_2_1_back inflection_2')
    time.sleep(1)
  

def grab_arm1_3_1_go():
    # 11.30 10:00 with Yang to define
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    ### maybe pre grab point
    #  utils.run_point(292.358,-354.835,61.116,-123.169,-1.433,-137.049,50.0) # 3-1工件

def grab_arm1_3_1_back(pre_point):
    # 11.30 10:00 with Yang to define
    print("in grab_arm1_3_1_back")
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2]+20,pre_point[3],pre_point[4],pre_point[5],20)
    print("in grab_arm1_3_1_back_mid")
    time.sleep(1)
    utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
    print("in grab_arm1_3_1_back done")
    

def grab_arm1_robot01_go():
    pass
    # time.sleep(1)
    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    # time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot01_back(pre_point):
    #to be sure......
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)

    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

def default():
    print('default')


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
    'arm1robot02': grab_arm1_robot01_go,
    'arm1robot03': grab_arm1_robot01_go,
    'arm1robot04': grab_arm1_robot01_go,
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
    'arm1robot02': grab_arm1_robot01_back,
    'arm1robot03': grab_arm1_robot01_back,
    'arm1robot04': grab_arm1_robot01_back,
}

def grab_go_switch_case(action_value):
    func = grab_go_options.get(action_value, default)
    return func()

def grab_back_switch_case(action_value,pre_point):
    func = grab_back_options.get(action_value, default)
    return func(pre_point)

#=========================================loosen
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
    utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    time.sleep(1)
    ## may be target loosen point
    ###utils.run_point(-174.378,420.125,140.000,160.045,2.924,44.804,10.0)
    time.sleep(1)
    
def loosen_arm1_robot02_back():
    # not sure about where to send,temp set robot02
    time.sleep(1)
    utils.run_point(-174.378,420.125,360.198,160.045,2.924,44.804,50.0)
    time.sleep(1)
    utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
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
    'arm1robot02': loosen_arm1_robot01_go,
    'arm1robot03': loosen_arm1_robot01_go,
    'arm1robot04': loosen_arm1_robot01_go,
    
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
    'arm1robot02': loosen_arm1_robot01_back,
    'arm1robot03': loosen_arm1_robot01_back,
    'arm1robot04': loosen_arm1_robot01_back,
}

def loosen_go_switch_case(action_value):
    func = loosen_go_options.get(action_value, default)
    return func()

def loosen_back_switch_case(action_value):
    func = loosen_back_options.get(action_value, default)
    return func()

app = Flask(__name__)
@app.route('/grab', methods=['POST','GET'])
def grab():
    print("###################in grab")
    global image_cb,show_img,lock,avg_depth,thread
    data = request.get_json()
    print(data)
    print(data['action'])
    print(data['param']['position'])
    if 'action' in data and 'param' in data :
        # action_value=data['action']
        action_value=data['param']['position']
        # action_value='arm1-1-1'
        param_value = data['param']
        print("###################",action_value,param_value)
        TEST=data['test']
        RUNNING=not(TEST)
        info=f"receive grab command:\n{action_value}\n{param_value}\n"
        log(info)

    if RUNNING:
        print('start arm_move service for grab')
        lock = threading.Lock()
        #########################
        rospy.Subscriber( '/camera_2/color/image_raw', Image,cb)
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

        print('adjusting grab rotate...... ')
        lock.acquire()
        image_cb = utils.rotate_angle
        lock.release()
        print('waiting1...... ')

        utils.wait_done()
        time.sleep(5.0)
        time.sleep(0.1)

        print('adjusting camera  to center...... ')
        lock.acquire()
        image_cb = utils.move_to_center
        lock.release()

        utils.wait_done()

        time.sleep(0.5)

        utils.wait_done()

        time.sleep(0.1)

        time.sleep(5.0)


        obj_depth = avg_depth
        print('obj_depth=',obj_depth)

        #shift to camera
        utils.shift_camera_1_1() # 1-1
        # utils.shift_camera_3_1() # 3-1

        ##############################################MODIFY_2-1
        # utils.shift_camera_2_1() # 2-1


        time.sleep(0.1)
        #if obj_depth > 0 :
        #shift depth 


        my_pose = pose()
        if obj_depth > 0 :
            my_pose.rz = obj_depth - 0.10
        else:
            my_pose.rz = 0.25

        print(my_pose)
        utils.shift_arm(my_pose,10.0)

        utils.arm_grap_catch()

        #=============================================================infection point
        print("========grab back inflection points......")
        grab_back_switch_case(action_value,pre_point=grab_pre_point)
        #=============================================================infection point

        # 3-1
        # utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1????
        
        
        print("going to standby point......")
        utils.go_back_home2()
        print("already at standby point......")
        print('grab DONE ^^')

        # rospy.signal_shutdown('demo running done')
        # thread.join()
        # info=f"finish grab command:\n{action_value}\n{param_value}\n\n"
        # log(info)

    if TEST:
        print("test grab successfully!")
        # time.sleep(4)
    return jsonify({'message': f'{action_value} Script executed on pc1 with param:{param_value} '})

@app.route('/loosen', methods=['POST','GET'])
def loosen():
    global image_cb,show_img,lock,avg_depth,thread
    data = request.get_json()
    if 'action' in data and 'param' in data :
        action_value=data['param']['position']
        param_value = data['param']
        TEST=data['test']
        RUNNING=not(TEST)
        info=f"loosen:\n{action_value}\n{param_value}\n"
        log(info)

    if RUNNING:
        #=============================================================infection point
        print("========loosen go inflection points......")
        loosen_go_switch_case(action_value)
        #=============================================================infection point
        

        print("going to pre_loosen point......")
        # utils.send_box_example_1_1()
        # loosen_pre_point=(np.float32(param_value["x"]),
        #                     np.float32(param_value["y"]),
        #                     np.float32(param_value["z"]),
        #                     np.float32(param_value["rx"]),
        #                     np.float32(param_value["ry"]),
        #                     np.float32(param_value["rz"]),
        #                     np.float32(param_value["speed"]))
        utils.run_point(np.float32(param_value["x"]),
                        np.float32(param_value["y"]),
                        np.float32(param_value["z"]),
                        np.float32(param_value["rx"]),
                        np.float32(param_value["ry"]),
                        np.float32(param_value["rz"]),
                        np.float32(param_value["speed"]))
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
        print('loosen DONE ^^')
        time.sleep(2)
        # rospy.signal_shutdown('loosen ok')
    if TEST:
        print("test loosen successfully!")

    return jsonify({'message': f'{action_value} Script executed on pc1 with param:{param_value} '})



if __name__ == '__main__':
    # app.run(host='0.0.0.0', port=5000)
    utils.all_init()
    # rospy.Subscriber( '/camera_2/color/image_raw', Image,cb)
    # rospy.Subscriber( '/camera_2/depth/image_raw', Image,depth_cb)
    # thread = threading.Thread(target = listen_msg)
    # thread.start()
    # utils.all_init()

    ####################
    
    app.run(host='192.168.1.226', port=7070)

