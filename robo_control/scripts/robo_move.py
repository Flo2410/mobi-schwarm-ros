import rospy
from geometry_msgs.msg import Twist
import can
import atexit
import numpy as np


can0 = None


def exit_handler():
    global can0
    stop()


def stop():
    # print("\033c", end="")
    # print("STOP!")
    msg2 = can.Message(arbitration_id=0x00002901, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    can0.send(msg2)


def twos(num):
    return np.uint16(~num + 1)


def drive(vx, vphi):
    """vx: m/s
    vphi: deg/s"""

    global can0

    # Vx
    val_x = vx / 0.0001
    val_x_int = np.int16(val_x)

    x_twos = twos(val_x_int)

    x_high = np.uint8(x_twos >> 8)
    x_low = np.uint8(x_twos)

    # Vphi
    val_phi = np.deg2rad(vphi) / 0.0001
    val_phi_int = np.int16(val_phi)

    phi_twos = twos(val_phi_int)

    phi_high = np.uint8(phi_twos >> 8)
    phi_low = np.uint8(phi_twos)

    msg = can.Message(arbitration_id=0x00002A01, data=[0x00, 0x00, x_low, x_high, phi_low, phi_high, 0x00, 0x00])  # To move laterally
    can0.send(msg)


def callback(data):
    rospy.loginfo(f"x: {data.linear.x} phi: {data.angular.z}")
    drive(-data.linear.x, np.rad2deg(data.angular.z))


def main():
    global can0

    atexit.register(exit_handler)

    can0 = can.interface.Bus(channel="can0", bustype="socketcan")

    rospy.init_node("theta_move", anonymous=False)
    rospy.Subscriber("/theta/cmd_vel", Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    main()
