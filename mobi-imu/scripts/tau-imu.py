import rospy
from geometry_msgs.msg import Quaternion
import adafruit_bno055
import board

imu_pub = None
initial_sensor = None


def do_loop(rate):
    global imu_pub

    qt = initial_sensor.quaternion

    try:
        imu_msg = Quaternion()
        imu_msg.w = -qt[0]
        imu_msg.x = qt[1]
        imu_msg.y = qt[2]
        imu_msg.z = qt[3]

        imu_pub.publish(imu_msg)
    except:
        rospy.logerr("error while reading imu data")
        rospy.loginfo(qt)

    rate.sleep()


def main():
    global imu_pub, initial_sensor, imu_pub_euler

    rospy.init_node("tau_imu", anonymous=False)
    imu_pub = rospy.Publisher("/tau/imu", Quaternion, queue_size=10)

    rate = rospy.Rate(10)

    i2c = board.I2C()

    # inital_sensor
    initial_sensor = adafruit_bno055.BNO055_I2C(i2c)

    while not rospy.is_shutdown():
        do_loop(rate)


if __name__ == "__main__":
    main()
