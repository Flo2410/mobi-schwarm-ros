# ----------------------------------------------------------------------------------------------------------------------------------------------
# File: pozyx-mqtt.py
# Created Date: Friday, December 9th 2022, 1:02:50 pm
# Author: Florian Hye <florian@hye.dev>
# Description: Fetch Mqtt Pozyx data and publish to ros net
# License: GNU GPLv3
# ----------------------------------------------------------------------------------------------------------------------------------------------


import paho.mqtt.client as mqtt
import ssl
import json
import rospy
from geometry_msgs.msg import Twist


host = "192.168.0.129"
port = 1883
topic = "tags"
username = ""
password = ""

theta_pub = None

theta_id = 0x687F
tau_id = 0x6A21


def on_connect(client, userdata, flags, rc):
    rospy.loginfo(mqtt.connack_string(rc))


# Callback triggered by a new Pozyx data packet
def on_message(client, userdata, msg):
    global theta_pub

    data = json.loads(msg.payload.decode())

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

        # rospy.loginfo(f"Positioning update: {id:<#6x} {pos} {ori}")


def on_subscribe(client, userdata, mid, granted_qos):
    rospy.loginfo("Subscribed to topic!")


def main():
    global theta_pub

    rospy.init_node("pozyx_mqtt", anonymous=False)
    theta_pub = rospy.Publisher("/theta/pozyx", Twist, queue_size=10)

    client = mqtt.Client(transport="tcp")
    client.username_pw_set(username, password=password)

    # sets the secure context, enabling the WSS protocol
    # client.tls_set_context(context=ssl.create_default_context())

    # set callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.connect(host, port=port)
    client.subscribe(topic)
    # works blocking, other, non-blocking, clients are available too.
    # client.loop_forever()
    client.loop_start()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    main()
