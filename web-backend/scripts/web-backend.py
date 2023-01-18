import rospy
import uuid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosbridge_msgs.msg import ConnectedClients
from std_msgs.msg import Empty

poses = []


def on_client(msg):
    rospy.Rate(0.5).sleep()
    update_poses()


def on_clear_path(msg):
    global poses
    poses = []
    update_poses()


def on_add_pose(msg):
    global poses
    poses.append(msg)
    update_poses()


def update_poses():
    global poses, path_pub
    pose_list = Path()

    pose_list.header.stamp = rospy.Time.now()
    pose_list.header.frame_id = str(uuid.uuid4())
    pose_list.poses = poses

    path_pub.publish(pose_list)


if __name__ == "__main__":
    global path_pub
    rospy.init_node("web_backend", anonymous=False)
    rospy.Subscriber("/web/add_pose", PoseStamped, on_add_pose, queue_size=1)
    rospy.Subscriber("/web/clear_path", Empty, on_clear_path, queue_size=1)

    rospy.Subscriber("/connected_clients", ConnectedClients, on_client, queue_size=1)

    path_pub = rospy.Publisher("/web/path", Path, queue_size=10)

    rospy.spin()
