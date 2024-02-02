import utils
import time
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
    time.sleep(1)
    utils.run_point(-280.844,-4.359,957.809,-89.715,0.197,19.012,50.0)
    time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot01_back(pre_point):
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)
    time.sleep(1)
    utils.run_point(-280.844,-4.359,957.809,-89.715,0.197,19.012,50.0)
    time.sleep(1)

def grab_arm1_robot02_go():
    print("grab_arm1_robot02_go start")
    time.sleep(1)
    utils.run_point(-331.425,-1.974,755.266,-172.486,0.688,85.688,50.0)
    time.sleep(1)
    # utils.run_point(-138.629,357.543,360.198,-179.764,-0.222,45.291,50.0)
    # time.sleep(1)

def grab_arm1_robot02_back(pre_point):
    #to be sure......
    time.sleep(1)
    utils.run_point(pre_point[0],pre_point[1],pre_point[2],pre_point[3],pre_point[4],pre_point[5],30)
    time.sleep(1)
    utils.run_point(-331.425,-1.974,755.266,-172.486,0.688,85.688,50.0)
    time.sleep(1)# time.sleep(1)
    # utils.run_point(-131.616,352.766,368.439,-178.767,-0.366,39.462,50.0)
    # utils.run_point(96.919,46.195,677.863,-94.069,1.221,48.456,50.0)
    time.sleep(1)

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
    utils.run_point(-491.788,284.279,308.577,179.192,-3.057,45.628,50.0)
    time.sleep(1)

def loosen_arm1_robot04_back():
    # not sure about where to send,temp set robot04
    time.sleep(1)
    utils.run_point(-491.788,284.279,308.577,179.192,-3.057,45.628,50.0)
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

