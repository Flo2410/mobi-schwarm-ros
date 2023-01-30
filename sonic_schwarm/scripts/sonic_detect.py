#!/usr/bin/env python

import time
import wiringpi
import atexit
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_msgs.msg import Bool
import rospy


def main():
    #pub = rospy.Publisher('/tau/sonic', Range, queue_size=1)
    pub_boom = rospy.Publisher('/tau/sonicboom', Bool, queue_size=1)
    rospy.init_node('sonic_detect', anonymous=False)
    rate = rospy.Rate(2) # 2hz

    wiringpi.wiringPiSetupGpio()

    triggerPin1 = 22
    echoPinN = 6
    echoPinS = 27
    wiringpi.pinMode(triggerPin1, 1)
    wiringpi.pinMode(echoPinN, 0)
    wiringpi.pinMode(echoPinS, 0)

    while not rospy.is_shutdown():
        s1 = tof(triggerPin1, echoPinN)
        s2 = tof(triggerPin1, echoPinS)

        #msg = Range()
        #msg.range = result
        #pub.publish(msg)

        msg_boom = Bool()

        if not s1 and not s2:
            msg_boom = False
        else:
            msg_boom = True

        pub_boom.publish(msg_boom)
        
        rate.sleep()

def tof(triggerPin, echoPin):
    wiringpi.digitalWrite(triggerPin, 1) 
    time.sleep(0.00001)
    wiringpi.digitalWrite(triggerPin, 0)

    while wiringpi.digitalRead(echoPin) == 0:
        pass

    signalStartTime = time.monotonic_ns()

    while wiringpi.digitalRead(echoPin) == 1:
        pass

    signalEndTime = time.monotonic_ns()
    duration = signalEndTime - signalStartTime
    meters = 343 * duration * 0.000000001
    result = meters / 2

    if (result < 0.60):
        msg_boom = True
    else:
        msg_boom = False

    return msg_boom


if __name__ == "__main__":
    main()