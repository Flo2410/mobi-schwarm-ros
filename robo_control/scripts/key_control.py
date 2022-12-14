import rospy
from geometry_msgs.msg import Twist
import atexit
from getkey import getkey
import numpy as np


SPEED_X = 0.1
SPEED_PHI = 60
SPEED_X_STEP = 0.05
SPEED_PHI_STEP = 5
MAX_SPEED_X = 0.85
MAX_SPEED_PHI = 100

DEBUG = True

pub = None


# register exit handler
def exit_handler():
    global can0
    stop()


def stop():
    # print("\033c", end="")
    print("STOP!")

    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)
    # msg2 = can.Message(arbitration_id=0x00002901, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    # can0.send(msg2)


def drive(vx, vphi, t):
    """vx: m/s
    vphi: deg/s
    t: sec"""

    rate = rospy.Rate(1)  # 1Hz

    vel_msg = Twist()
    vel_msg.linear.x = vx
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = np.deg2rad(vphi)

    tt = t
    while tt > 0:
        pub.publish(vel_msg)
        rate.sleep()
        tt -= 1

    stop()

    rospy.loginfo("published a message")


last_vx = 0
last_vphi = 0


def key_event(key):
    global last_vx, last_vphi, SPEED_X, SPEED_PHI, SPEED_X_STEP, SPEED_PHI_STEP

    print("\033c", end="")

    if DEBUG:
        print(f"name: {key}")
        print("")

    if key == "q":
        exit(0)

    vx = 0
    vphi = 0

    # if key == "s":
    #     vx += 1
    # elif key == "w":
    #     vx -= 1
    # elif key == "a":
    #     vphi -= 1
    # elif key == "d":
    #     vphi += 1
    # elif key == " ":
    #     vx = 0
    #     vphi = 0

    vx *= SPEED_X
    vphi *= SPEED_PHI

    if key == "y" and SPEED_X - SPEED_X_STEP > SPEED_X_STEP:
        SPEED_X -= SPEED_X_STEP
    elif key == "x" and SPEED_X + SPEED_X_STEP <= MAX_SPEED_X:
        SPEED_X += SPEED_X_STEP
    elif key == "c" and SPEED_PHI - SPEED_PHI_STEP > 0:
        SPEED_PHI -= SPEED_PHI_STEP
    elif key == "v" and SPEED_PHI + SPEED_PHI_STEP <= MAX_SPEED_PHI:
        SPEED_PHI += SPEED_PHI_STEP

    print(f"{'SPEED X':<10}: {SPEED_X:<6.3f} m/s | {'SPEED X STEP':<15}: {SPEED_X_STEP:<6.3f}")
    print(f"{'SPEED PHI':<10}: {SPEED_PHI:<6.3f} deg/s | {'SPEED PHI STEP':<15}: {SPEED_PHI_STEP:<6.3f}")
    print("")

    if key == "W":
        vx = SPEED_X
        vphi = 0
        t = 0.3 / SPEED_X
        print(f"vx: {vx:>7.3f} | vphi: {vphi:>7.3f}  | t: {t}")
        print("")
        drive(vx, vphi, t)
        return
    elif key == "S":
        vx = -SPEED_X
        vphi = 0
        t = 0.3 / SPEED_X
        print(f"vx: {vx:>7.3f} | vphi: {vphi:>7.3f}  | t: {t}")
        print("")
        drive(vx, vphi, t)
        return
    elif key == "D":
        vx = 0
        vphi = -SPEED_PHI
        t = 30 / SPEED_PHI
        print(f"vx: {vx:>7.3f} | vphi: {vphi:>7.3f}  | t: {t}")
        print("")
        drive(vx, vphi, t)
        return
    elif key == "A":
        vx = 0
        vphi = SPEED_PHI
        t = 30 / SPEED_PHI
        print(f"vx: {vx:>7.3f} | vphi: {vphi:>7.3f}  | t: {t}")
        print("")
        drive(vx, vphi, t)
        return

    if key == "k" or key == "l":
        vx = SPEED_X
        u = 2 * np.pi * 0.5
        t = u / SPEED_X

        vphi = -360 / t

        if key == "k":
            vphi *= -1

        print(f"vx: {vx:>7.3f} | vphi: {vphi:>7.3f} | t: {t}")
        print("")
        drive(vx, vphi, t)
        return

    print(f"vx: {vx:>7.3f} | vphi: {vphi:>7.3f}")
    print("")

    if last_vx == vx and last_vphi == vphi:
        return

    # drive(vx, vphi)

    last_vx = vx
    last_vphi = vphi


def main():
    global pub
    atexit.register(exit_handler)

    # CAN-Interface initialisieren
    # read_serial_number()

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.init_node("key_control", anonymous=True)

    while True:
        key = getkey()
        key_event(key)


if __name__ == "__main__":
    main()
