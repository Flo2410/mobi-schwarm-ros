import rospy
from geometry_msgs.msg import Twist
import numpy as np
import atexit

t_x = 3116
t_y = 3115

t = np.array([t_x, t_y])

theta_pub = None
finish = False


def stop():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    theta_pub.publish(vel_msg)


def exit_handler():
    stop()


def callback(data):
    global theta_pub, finish
    p = np.array([data.linear.x, data.linear.y])  # current pos
    phi = data.angular.z  # current rotation

    rospy.loginfo(f"x: {p[0]} mm, y: {p[1]} mm, phi: {np.rad2deg(phi)} deg, phi: {phi} rad")

    vec = t - p
    t_dir = np.arctan2(*vec) + np.pi  # target dir
    dist = np.linalg.norm(vec)

    rospy.loginfo(f"t_dir: {np.rad2deg(t_dir)} deg, t_dir: {t_dir} rad, dist: {dist} mm")

    omega = t_dir - phi  # angle diverance

    rospy.loginfo(f"omega: {np.rad2deg(omega)} deg, omega: {omega} rad")

    if dist < 150:
        finish = True

    if finish:
        stop()
        return

    vel_msg = Twist()
    vel_msg.linear.x = 0

    if np.abs(np.rad2deg(omega)) < 5:
        vel_msg.linear.x = 0.2

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    if omega > 5:
        vel_msg.angular.z = -0.40

    if omega < 180:
        vel_msg.angular.z *= -1

    theta_pub.publish(vel_msg)


def main():
    global theta_pub
    atexit.register(exit_handler)

    rospy.init_node("theta_drive", anonymous=True)
    theta_pub = rospy.Publisher("/theta/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/theta/pozyx", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    main()
