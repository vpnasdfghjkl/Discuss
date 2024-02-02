scripts/robot.py 法奥机械臂的demo，提供了一个ROS服务move_arm  可以让机械臂到达某个位姿，传入参数是x y z r p y   xyz单位是mm  rpy是角度°
                         需要通过网线与机械臂连接，并且网段需要为192.168.58.xxx  机械臂固定IP为192.168.58.2   8080端口号
scripts/find_object.py 物体特征识别demo 需要与capture_roi.py 联合用 需要修改相机相关的话题 以及传入object的图片路径  拿到的是相机和物体的相关关系
scripts/grap.py  机械爪的串口控制demo
scripts/woosh_demo.py 小车控制demo，给小车发送一个任务，当小车到达点位后停下
scripts/capture_roi.py 截图用的工具 里面订阅的相机话题需要修改 截下来的图用作find_object.py的输入图片





