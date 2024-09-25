#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Global variables
bridge = CvBridge()
screenwidth = 800
screenheight = 800

from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)


def image_callback(data):
    """Callback function to process the camera image."""
    global bridge

    try:
        # Convert the ROS Image message to an OpenCV image (BGR format)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        # Display the image using OpenCV
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret2, filtered = cv2.threshold(gray,140, 255, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur(filtered,(5,5),0.7)

        prev = 255
        crossings = []

        #check for a change in brightness
        for index in range(screenwidth):
            pixel = blur[600][index]

            if ((pixel == 255 or pixel == 0) and pixel != prev):
                crossings.append(index)
                blur[600][index] = 127
            prev = pixel

        if len(crossings) == 0:
            crossings = [200, 200]
        output = cv2.circle(cv_image, (int(sum(crossings)/len(crossings)), 600), 20, (255),-1)
        rospy.loginfo(crossings)
        cv2.imshow("Gazebo Camera Image", output)

        move = Twist()
        move.linear.x = 0.2
        if (int(sum(crossings)/len(crossings))-400 < 0):
            move.angular.z = 0.9*(abs(int(sum(crossings)/len(crossings))-400))/400
        else:
            move.angular.z = -0.9*(abs(int(sum(crossings)/len(crossings))-400))/400
        pub.publish(move)
        cv2.waitKey(1)  # Add a small delay to allow image display

    except CvBridgeError as e:
        rospy.logerr(f"Error converting ROS Image message to OpenCV: {e}")

def main():
    # Subscribe to the Gazebo camera topic (Modify the topic based on your setup)
    rospy.Subscriber('/rrbot/camera1/image_raw', Image, image_callback)

    rospy.loginfo("Gazebo camera reader node initialized. Waiting for camera images...")

    # Keep the node running until shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Gazebo camera reader node.")
    finally:
        # Destroy OpenCV windows when the node is shut down
        cv2.destroyAllWindows()