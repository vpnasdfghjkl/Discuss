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

avg_depth = None

image_cb = None
lock = None

show_img = None



def cb(color_image):
    global image_cb,show_img
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



# rospy.init_node()
utils.all_init()

lock = threading.Lock()

##############
rospy.Subscriber( '/camera_2/color/image_raw', Image,cb)
rospy.Subscriber( '/camera_2/depth/image_raw', Image,depth_cb)
######################
thread = threading.Thread(target = listen_msg)
thread.start()


time.sleep(5.0)

#goto start point
print('demo start')
utils.go_back_home2()

#low 
print("to_A")
utils.run_point(297.787,-325.616,670.058,-89.181,-0.486,-137.505,50.0) # 1-1工件
# utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件
# utils.run_point(292.358,-354.835,61.116,-123.169,-1.433,-137.049,50.0) # 3-1工件

##############################################MODIFY_2-1
# utils.run_point() # 2-1工件

print('goto start point ok ')

lock.acquire()

image_cb = utils.rotate_angle

lock.release()

utils.wait_done()


time.sleep(5.0)


time.sleep(0.1)

lock.acquire()

image_cb = utils.move_to_center

lock.release()

utils.wait_done()

time.sleep(0.1)

utils.wait_done()

time.sleep(0.1)

time.sleep(5.0)


obj_depth = avg_depth
print(obj_depth)

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

# 3-1工件
# utils.run_point(188.089,-315.239,256.008,-108.104,-0.721,-137.456,50.0) # 3-1工件

utils.go_back_home2()

# 1-1工件
utils.send_box_example_1_1()

# 3-1工件
# utils.send_box_example_3_1()

##############################################MODIFY_2-1
# utils.send_box_example_2_1()


utils.arm_grap_release()

utils.send_box_release_on()

utils.go_back_home2()

print('demo done')

time.sleep(2)
rospy.signal_shutdown('demo running done')

thread.join()


