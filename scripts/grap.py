import serial
import time

def serial_init():
    # 打开串口
    ser = serial.Serial('/dev/armjaw', 115200)
    return ser

def grap_init(ser):
    data = bytes.fromhex("09 10 03 E8 00 01 02 00 01 24 78")
    ser.write(data)  
    ser.timeout = 1 
    data = []
    for i in range(8):
        data.append(ser.read(1))
    if len(data) != 8:
        print('response error..')
        print(data)
    print('response is ')
    print(data)

def grap_release(ser):
    data = bytes.fromhex("09 10 03 E8 00 03 06 00 09 00 00 FF FF AE 81")
    ser.write(data)
    ser.timeout = 1 
    data = []
    for i in range(8):
        data.append(ser.read(1))
    if len(data) != 8:
        print('response error..')
    print('response is ')
    print(data)

def grap_catch(ser):
    data = bytes.fromhex("09 10 03 E8 00 03 06 00 09 FF 00 FF FF 9E 95")
    ser.write(data)   
    ser.timeout = 1 
    data = []
    for i in range(8):
        data.append(ser.read(1))
    if len(data) != 8:
        print('response error..') 
    print('response is ')
    print(data)

def get_grap_loc(ser):
    data = bytes.fromhex("09 04 07 D1 00 01 61 CF")
    ser.write(data)
    ser.timeout = 1
    data = []
    for i in range(7):
        data.append(ser.read(1))
    print('response is ')
    print(data)

    
    if len(data) ==  7 and data[0] == b'\t' and data[1] == b'\x04' and data[2] == b'\x02':
        loc = ord(data[3])
        if loc > 250 :
            return True
        elif loc < 10:
            return False
        else:
            print('here {}'.format(loc))
            return None
    else:
        print('serial error.')
        return None



if __name__ == "__main__":

    test_time = 10 
    ser = serial_init()
    grap_init(ser)
    #wait for init done..
    time.sleep(3)
    #grap_catch(ser)
    grap_release(ser)
    get_grap_loc(ser)
    '''
    for i in range(test_time):
        #grap_catch(ser)
        time.sleep(5)
        res = get_grap_loc(ser)
        if res is None:
            print('error')
        if res is False:
            print('control error')
            continue
        grap_release(ser)
        time.sleep(5)
        res = get_grap_loc(ser)
        if res is None:
            print('error')
        if res is True:
            print('control error')
            continue        
    '''
    # 关闭串口
    ser.close()
