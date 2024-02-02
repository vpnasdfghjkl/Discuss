import socket
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped,PoseStamped
import math
from tf.transformations import  quaternion_from_euler
from demo.srv import robot_move
from demo.msg import pose
from threading import Lock
import time
import frrpc
import threading
from std_srvs.srv import Empty,EmptyResponse
import ctypes





class robot_packet(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("program_state", ctypes.c_ubyte * 1),
                ("error_code", ctypes.c_ubyte*1),
                ("robot_mode",ctypes.c_ubyte * 1),
                ("not_used",ctypes.c_ubyte*225),
                ("emergency_stop",ctypes.c_ubyte * 1),
                ("move_done",ctypes.c_int32 * 1),
                ("_not_used_",ctypes.c_ubyte*1)
                ]
    



def get_data(client_socket):
    received_data = client_socket.recv(1024)

    get_sum = received_data[-2:]
    high = get_sum[1] << 8
    low = get_sum[0]
    get_sum = high + low
        #print(hex(get_sum))

    data = received_data[:-2]
    check_sum = 0 
    for i in data:
        check_sum += i

    if check_sum == get_sum:
        #print('ok')

        if len(received_data[5:-2]) == 234:
            data = robot_packet.from_buffer_copy(received_data[5:-2])

            #print('error_code :',data.error_code[0])
            #print('move_done :',data.move_done[0])
            return data
        else:
            return None
    else:
        return None









lock = Lock()


thread1 = None
app_running_flag = True


robot = None
robot_x = 0.0
robot_y = 0.0
robot_z = 0.0
robot_r = 0.0
robot_p = 0.0
robot_y_ = 0.0
pub = None

packet = robot_packet()

def robot_shut_down():
    global app_running_flag 
    app_running_flag = False
    #rospy.logerr('robot shut down')
    #robot.StopMotion()
    if thread1 is not None:
        thread1.join()


def move_cancel(req):
    rospy.logerr('stop move...')
    robot.StopMotion()
    return EmptyResponse()

def arm_move(point,speed):
    with lock:
        print('start move.')
        robot.MoveCart(point,0,0,speed,100.0,100.0,-1.0,-1)
        print('move done.')
    return



def move_handler(request):
    global robot_x,robot_y,robot_z,robot_r,robot_p,robot_y_
    #arm_move(request.robot_x,request.robot_y,request.robot_z,request.robot_r,request.robot_p,request.robot_y_)
    #print('move done ...' , packet.move_done[0])
    print('move call')
    speed = round(request.speed,0)
    x = request.Pose.rx * 1000
    y = request.Pose.ry * 1000
    z = request.Pose.rz * 1000
    rr = request.Pose.r 
    rp = request.Pose.p
    ry = request.Pose.y

    data = [x,y,z,rr,rp,ry]
    if request.shift:
        print('arm shift')
        robot_pose = [robot_x * 1000 , robot_y * 1000, robot_z * 1000,robot_r,robot_p,robot_y_]
        with lock:
            joint_pose = robot.GetInverseKin(0,robot_pose , -1)
            time.sleep(0.1)
        if joint_pose[0] == 0 :
            joint_pose = joint_pose[1:]
            ep = [0.0,0.0,0.0,0.0]
            #data[5] = 90.0
            print(data)

            with lock:
                robot.MoveL(joint_pose,robot_pose,0,0,speed , 100.0,100.0,-1.0,ep,0,2,data)
                time.sleep(0.1)
        else:
            return False


    else:
        print('arm move')
        arm_move(data,speed)
        rospy.loginfo("X: %f, Y: %f  Z: %f R: %f P: %f Y: %f speed: %f", data[0],data[1], data[2] ,data[3] ,data[4] ,data[5],speed)
    return True


def get_robot_pose(robot):
    global robot_x,robot_y,robot_z,robot_r,robot_p,robot_y_
    ret = robot.GetActualTCPPose(0)
    if ret[0] == 0 :
        robot_x = float(ret[1]) / 1000
        robot_y = float(ret[2]) / 1000
        robot_z = float(ret[3]) / 1000
        robot_r = float(ret[4])
        robot_p = float(ret[5])
        robot_y_= float(ret[6])
    else:
        print(ret)
        print('get pose error..')



def send_robot_base_transform(broadcaster):
    t = TransformStamped()

    # 填充变换数据
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'  # 使用'map'作为世界坐标系的父坐标系
    t.child_frame_id = 'robot_base'
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    broadcaster.sendTransform(t)
    return 

def send_robot_tool(broadcaster):
    t = TransformStamped()

    roll = math.radians(robot_r)   # 将角度转换为弧度
    pitch = math.radians(robot_p) # 将角度转换为弧度
    yaw = math.radians(robot_y_)     # 将角度转换为弧度
    quat = quaternion_from_euler(roll, pitch, yaw)

    # 填充变换数据
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'robot_base'  
    t.child_frame_id = 'robot_tool'
    t.transform.translation.x = robot_x
    t.transform.translation.y = robot_y
    t.transform.translation.z = robot_z
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]


    arm_pose = PoseStamped()
    arm_pose.pose.position.x = robot_x
    arm_pose.pose.position.y = robot_y
    arm_pose.pose.position.z = robot_z
    
    arm_pose.pose.orientation.x = quat[0]
    arm_pose.pose.orientation.y = quat[1]
    arm_pose.pose.orientation.z = quat[2]
    arm_pose.pose.orientation.w = quat[3]
    pub.publish(arm_pose)
    
    broadcaster.sendTransform(t)



def send_camera_pose(broadcaster):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'robot_tool'  
    t.child_frame_id = 'camera_pose'
    t.transform.translation.x = -0.013
    t.transform.translation.y = -0.10
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    broadcaster.sendTransform(t)


def robot_msg_thread():
    global packet
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 固定的
    host = '169.254.128.88'
    port = 8083
    print('robot msg thread start !!!')
    try:
        client_socket.connect((host, port))
    except:
        exit(0)
    
    while app_running_flag:
        check_packet = get_data(client_socket)
        if check_packet is not None:
            packet = check_packet
    print('robot msg thread exit..')

def cancel_thread():
    rospy.Service('arm_cancel', Empty, move_cancel)
    rospy.spin()


if __name__ == "__main__":
    #固定的IP
    robot = frrpc.RPC('169.254.128.88')
    ret = 0
    try:
        ret = robot.GetSDKVersion()
        print('robot version is {}'.format(ret[1]))
    except:
        exit(0)

    rospy.init_node('robot_control')
    pub = rospy.Publisher('arm_pose',PoseStamped,queue_size = 10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(5)
    service = rospy.Service('move_arm', robot_move, move_handler)
    
    rospy.on_shutdown(robot_shut_down)

    #开始机器人消息监听线程
    # 法奥sdk有些问题 暂时不能通过接口停止机械臂
    #thread1 = threading.Thread(target=robot_msg_thread)
    #thread1.start()

    #thread2 = threading.Thread(target=cancel_thread)
    #thread2.start()






    while not rospy.is_shutdown():
        with lock:
            try:
                get_robot_pose(robot)
            except:
                rospy.logerr('get robot pose timeout..')
        send_robot_base_transform(tf_broadcaster)
        send_robot_tool(tf_broadcaster)
        send_camera_pose(tf_broadcaster)
        rate.sleep()

    rospy.spin()
