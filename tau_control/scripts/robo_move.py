#!/usr/bin/env python3

import can
import atexit
import numpy as np
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32

can0 = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')

last_callback_time = 0
not_first_callback = False
time_since_last_callback = 0
qr_info = ''
b_drive = False
boom = False


def callback_boom(data):
    # boolean value in attribute data
    global boom
    boom = data.data

def callback_pos(data):
    #global not_first_callback
    #global last_callback_time
    #global time_since_last_callback
    global qr_info
    global b_drive
    global boom

    # Check time since last callback
    #callback_time = rospy.get_time()
    #if not_first_callback:
    #    time_since_last_callback = rospy.get_time() - last_callback_time
    #    print(time_since_last_callback)

    # boolean value in attribute data
    if data.data and b_drive and not boom:
        # print(data.data)
        if qr_info == 'back':
            vel = 0.05
            rps = 0.1
        elif qr_info == 'front':
            vel = -0.1
            rps = 0.25
        elif qr_info == 'none':
            vel = 0.15
            rps = 0.25
        else:
            vel = 0
            rps = 0

        h,l = complement(vel)
        h1, l1 = complement(rps)
        h2, l2 = complement(-rps)

        #buff = 0.082
        #print(qr_info)
        #print(data.data)

        if data.data <= 95 and data.data >= 65 and qr_info != 'stop': #time_since_last_callback <= buff:
            msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, l, h, 0x00, 0x00, 0x00, 0x00], extended_id=True)
        elif data.data < 65 and qr_info != 'stop': # time_since_last_callback <= buff:
            msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, l, h, l2, h2, 0x00, 0x00], extended_id=True)
        elif data.data > 95 and qr_info != 'stop': # time_since_last_callback <= buff:
            msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, l, h, l1, h1, 0x00, 0x00], extended_id=True)
        else:
            msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], extended_id=True)
        can0.send(msg)
    else:
        msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], extended_id=True)
        can0.send(msg)

    #not_first_callback = True
    #last_callback_time = callback_time

def callback_info(data):
    global qr_info
    qr_info = data.data
    #print(data.data)

def callback_drive(data):
    global b_drive
    b_drive = data.data

def listener():
    rospy.init_node('robo_move', anonymous=False)
    #rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
    rospy.Subscriber('/QR_Text', String, callback_info)
    rospy.Subscriber('/QR_Pos', Float32, callback_pos)
    rospy.Subscriber('/tau/sonicboom', Bool, callback_boom)
    rospy.Subscriber('/theta/drive', Bool, callback_drive)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def complement(speed):
    speed = speed / 0.0001
    speed = np.int16(speed)

    speed = np.uint16(~speed + 1)

    hbyte = np.uint8(speed >> 8)
    lbyte = np.uint8(speed)

    return hbyte, lbyte

def stopper():
    print("################################## stopped ##################################")
    msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], extended_id=True)
    can0.send(msg)


#########################################################################

atexit.register(stopper)
listener()
