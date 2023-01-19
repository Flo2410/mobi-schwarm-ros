import rospy
from enum import Enum
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import numpy as np
import atexit

phi = 0
pos = Twist()
do_drive = False
path = Path()
target = np.array([])
current_target_index = -1

theta_pub = None
theta_status_pub = None


class Level(Enum):
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


class TurnDir(Enum):
    LEFT = -1
    RIGHT = 1

    def flip(self):
        if self == TurnDir.LEFT:
            return TurnDir.RIGHT
        else:
            return TurnDir.LEFT


def euler_from_quaternion(w, x, y, z):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def stop():
    global do_drive
    do_drive = False

    key_vals = [KeyValue("do_drive", f"{do_drive}")]
    publish_status(Level.OK, key_vals)

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


def publish_status(level: Level, key_values):
    global theta_status_pub

    status = DiagnosticStatus()
    status.hardware_id = "mobi-theta"
    status.name = "Theta Drive"
    status.message = "Driving status"
    status.level = level.value
    status.values = key_values

    theta_status_pub.publish(status)


def next_target():
    global path, target, current_target_index
    if len(path.poses) - 1 > current_target_index:
        current_target_index += 1
        x = path.poses[current_target_index].pose.position.x
        y = path.poses[current_target_index].pose.position.y
        target = np.array([x, y])
        key_vals = [KeyValue("current_target_index", f"{current_target_index}"), KeyValue("target_pos", f"x: {x} mm  y: {y} mm")]
        publish_status(Level.OK, key_vals)
    else:
        rospy.loginfo("no more points")
        stop()


def callback_drive(data: Bool):
    global do_drive, target, path, current_target_index
    if data.data:
        current_target_index = -1
        next_target()
        do_drive = True

        key_vals = [KeyValue("do_drive", f"{do_drive}")]
        publish_status(Level.OK, key_vals)
    else:
        stop()


def callback_path(data: Path):
    global path
    path = data


def callback_imu(data: Quaternion):
    global phi
    euler = euler_from_quaternion(data.w, data.x, data.y, data.z)
    phi = euler[2] + np.pi


def callback_pozyx(data: Twist):
    global pos
    pos = np.array([data.linear.x, data.linear.y])  # current pos


def drive_loop():
    global theta_pub, phi, pos, do_drive, target
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if not do_drive:
            continue

        vec = target - pos
        t_dir = np.arctan2(*vec) + np.pi  # target dir
        dist = np.linalg.norm(vec)

        omega = t_dir - phi  # angle differance

        if dist < 250:  # arrived at target
            rospy.loginfo(f"arrived at point with index {current_target_index}")
            next_target()
            continue

        vel_msg = Twist()
        vel_msg.linear.x = 0

        if np.abs(np.rad2deg(omega)) < 10:
            vel_msg.linear.x = 0.2

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Drehrichtung
        turn_dir = TurnDir.RIGHT  # default dir

        if omega < 0:
            turn_dir = TurnDir.LEFT

        flip = False
        if np.abs(np.rad2deg(omega)) > 180:
            flip = True
            turn_dir = turn_dir.flip()

        rot_speed = 0.4

        if np.abs(np.rad2deg(omega)) < 5:
            rot_speed = 0.2

        if np.abs(np.rad2deg(omega)) > 3:
            vel_msg.angular.z = rot_speed * turn_dir.value

        rospy.loginfo(f"phi: {np.rad2deg(phi)} deg, t_dir: {np.rad2deg(t_dir)} deg, omega: {np.rad2deg(omega)} deg, turn_dir: {turn_dir.name}, dist: {dist} mm")
        key_vals = [
            KeyValue("distance", f"{np.format_float_positional(dist, 2)} mm"),
            KeyValue("phi", f"{np.format_float_positional(np.rad2deg(phi), 2)} deg"),
            KeyValue("omega", f"{np.format_float_positional(np.rad2deg(omega), 2)} deg"),
            KeyValue("target_direction", f"{np.format_float_positional(np.rad2deg(t_dir), 2)} deg"),
            KeyValue("turn_direction", turn_dir.name),
            KeyValue("flip", str(flip)),
        ]
        publish_status(Level.OK, key_vals)

        theta_pub.publish(vel_msg)
        rate.sleep()


def main():
    global theta_pub, theta_status_pub
    atexit.register(exit_handler)

    rospy.init_node("theta_drive", anonymous=False)
    theta_pub = rospy.Publisher("/theta/cmd_vel", Twist, queue_size=10)
    theta_status_pub = rospy.Publisher("/theta/status", DiagnosticStatus, queue_size=10)

    rospy.Subscriber("/theta/pozyx", Twist, callback_pozyx)
    rospy.Subscriber("/theta/imu", Quaternion, callback_imu)
    rospy.Subscriber("/theta/drive", Bool, callback_drive)
    rospy.Subscriber("/web/path", Path, callback_path)

    drive_loop()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == "__main__":
    main()
