import numpy as np
from sensor_msgs.msg import Image
import rospy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String


class Detector():

    def __init__(self):
        self.bridge = CvBridge()
        self.img_topic = '/camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(self.img_topic, Image, self.callback)
        self.pub_topic = '/witsdetector'
        self.pub = rospy.Publisher(self.pub_topic, String, queue_size=10)
        self.image = None
        self.image_received = False

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

        self.height = data.height
        self.width = data.width

    def get_view(self):
        if self.image_received:
            plt.imshow(self.image)
            plt.show()

    def get_middle(self):
        padding = 30
        middle = self.height // 2 + 5
        return self.image[middle - padding:middle + padding, :self.width]

    def get_threshold(self):
        hsv_img = cv2.cvtColor(self.get_middle(), cv2.COLOR_BGR2HSV)
        self.binary_img = cv2.inRange(hsv_img, (60, 100, 10), (60, 255, 255))

    def is_present(self):
        return np.any(self.binary_img)