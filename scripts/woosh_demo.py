import ctypes 
import time



lib = ctypes.CDLL('librobot_demo.so')

lib.woosh_robot_init.argtypes = [ctypes.c_char_p]
#ip 地址需要修改
ip_addr = '172.20.12.104'
ip_addr = ip_addr.encode()
c_ip_addr = ctypes.c_char_p(ip_addr)

#连接底盘
lib.woosh_robot_init(c_ip_addr)


lib.woosh_robot_debug.argtypes = [ctypes.c_int]
control = ctypes.c_int(0)
#打开debug开关
lib.woosh_robot_debug(control)


#下发任务
lib.woosh_robot_run_task_group.argtypes = [ctypes.c_int]
#任务组id  需要修改
task_id = ctypes.c_int(1081947484)
res = lib.woosh_robot_run_task_group(task_id)
print(res)


lib.woosh_robot_listen.argtypes = [ctypes.c_char_p,ctypes.c_char_p]
#目标点名称 需要修改
dest_name = '123'
dest_name = dest_name.encode()
dest_name = ctypes.c_char_p(dest_name)

# 监听是否到到等待状态  可修改
dest_state = 'ActionWait'
dest_state = dest_state.encode()
dest_state = ctypes.c_char_p(dest_state)


#监听目标点 123  状态wait.  当底盘状态为 在目标点123等待的时候返回 --- 阻塞
lib.woosh_robot_listen(dest_name , dest_state)

# ...........

#任务完成。 打破等待 
res = lib.woosh_robot_break_wait()
print(res)



