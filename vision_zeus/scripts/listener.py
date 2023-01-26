#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import String

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

qcd = cv2.QRCodeDetector()

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('ImageRecog', anonymous=False)

QRPos = rospy.Publisher('/QR_Pos', Float32, queue_size=10)
QRtext = rospy.Publisher('/QR_Text', String, queue_size=10)

#####

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):

    # Initialize the CvBridge class
    bridge = CvBridge()
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    
    
    
    
    
    info, points, code = qcd.detectAndDecode(cv_image)
    if info == 'front' or info =='back':
        x1 = points[0][0][0]
        x2 = points[0][1][0]
        x3 = points[0][2][0]
        x4 = points[0][3][0]
        
        x_one = (x1 + x2)/2
        x_two = (x3 + x4)/2
        x = (x_one + x_two)/2

        QRPos.publish(x)
        cv_image = cv2.polylines(cv_image, [points.astype(int)], True, [255,0,0], 8)
        
        if info == 'front':
            QRtext.publish(info)
        else:
            QRtext.publish(info)     
    
    else:
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #58 156 190
        #[22, 93, 50]
        lower_yellow = np.array([22, 93, 50])
        #, dtype = "uint8")
        upper_yellow = np.array([35, 255, 255])
        #, dtype = "uint8")
        
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        check = np.sum(mask)
        pixel = cv2.findNonZero(mask)
        
        #print(check)
        if check > 8000:
            pos = np.mean(pixel[:,:,0])
        
            QRPos.publish(pos)
            QRtext.publish("none")
        else:
            QRtext.publish("stop")
            QRPos.publish(1.0)
            
            
        cv2.imshow("Result Mask", mask)
        cv2.imshow("Result RGB", cv_image)
        c = cv2.waitKey(1)
    


# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/VideoRaw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
