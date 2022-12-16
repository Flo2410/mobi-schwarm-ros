# ----------------------------------------------------------------------------------------------------------------------------------------------
# File: pozyx-mqtt.py
# Created Date: Friday, December 9th 2022, 1:02:50 pm
# Author: Florian Hye <florian@hye.dev>
# Description: Fetch Mqtt Pozyx data and publish to ros net
# License: GNU GPLv3
# ----------------------------------------------------------------------------------------------------------------------------------------------


import rospy
from geometry_msgs.msg import Twist
import random

theta_pub = None

theta_id = 0x687F
tau_id = 0x6A21


# Callback triggered by a new Pozyx data packet
def on_message():
    global theta_pub, data
    data = [
        {
            "version": "1",
            "tagId": "26751",
            "timestamp": 1670581829.141,
            "success": True,
            "data": {
                "coordinates": {"x": random.randint(1000, 1100), "y": random.randint(2200, 2300), "z": 1799},
                "orientation": {"yaw": random.uniform(-3.5, 3.5), "roll": random.uniform(-3.5, 3.5), "pitch": random.uniform(-3.5, 3.5)},
                "tagData": {},
                "metrics": {"latency": 52, "rates": {"success": 16.063, "update": 16.063}},
                "zones": [],
            },
        }
    ]

    for d in data:
        id = int(d.get("tagId"))

        if not id == theta_id and not id == tau_id:
            continue

        if not d.get("success"):
            rospy.logwarn(d.get("errorCode"))
            continue

        pos = d.get("data").get("coordinates")
        ori = d.get("data").get("orientation")

        pose = Twist()
        pose.linear.x = pos.get("x")
        pose.linear.y = pos.get("y")
        pose.linear.z = pos.get("z")
        pose.angular.x = ori.get("roll")
        pose.angular.y = ori.get("pitch")
        pose.angular.z = ori.get("yaw")

        theta_pub.publish(pose)

        rospy.loginfo(f"Positioning update: {id:<#6x} {pos} {ori}")


def main():
    global theta_pub

    rospy.init_node("pozyx_fake", anonymous=False)
    theta_pub = rospy.Publisher("/theta/pozyx", Twist, queue_size=10)

    rate = rospy.Rate(1)  # 2hz
    while not rospy.is_shutdown():
        on_message()
        rate.sleep()


if __name__ == "__main__":
    main()
