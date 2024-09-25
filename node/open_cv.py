import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Twist

linX = 0.5
angZ = 0
screenwidth = 800
screenheight = 800

def image_callback(msg):
    # Convert ROS Image message to OpenCV format
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Display the image
    cv.imshow("Camera Feed", cv_image)
    cv.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('camera_reader', anonymous=True)
    rospy.Subscriber('/rrbot/camera1/image_raw', Image, image_callback)
    rospy.spin()

    # Close all OpenCV windows
    cv.destroyAllWindows()

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = linX
move.angular.z = 0.5

while not rospy.is_shutdown():
   pub.publish(move)
   rate.sleep()

def distFromCenter(cv_image):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret2, filtered = cv.threshold(gray,95, 255, cv.THRESH_BINARY)
    blur = cv.GaussianBlur(filtered,(5,5),0.7)

    prev = 255
    crossings = []

    #check for a change in brightness
    for index in range(screenwidth-20):
      pixel = blur[screenwidth-20][index]

      if ((pixel == 255 or pixel == 0) and pixel != prev):
         crossings.append(index)
         blur[screenwidth-20][index] = 127
      prev = pixel

    if len(crossings) == 0:
      crossings = [-400, -400]
    
    center = (int(sum(crossings)/len(crossings)))
    return screenwidth/2-center