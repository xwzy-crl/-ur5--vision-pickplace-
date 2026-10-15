#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("vision_practice",anonymous=True)

message=rospy.wait_for_message(
    "/camera/color/image_raw",
    Image,
    timeout=10.0
)
bridge = CvBridge()
image = bridge.imgmsg_to_cv2(message,desired_encoding="bgr8")
height, width, channels = image.shape

print("高度：", height)
print("宽度：", width)
print("通道数：", channels)

u = width // 2
v = height // 2

print("中心像素 BGR：", image[v, u])

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

print("中心像素 HSV：", hsv[v, u])
mask_low = cv2.inRange(
    hsv,
    (0,90,50),
    (10,255,255),
)
mask_high = cv2.inRange(
    hsv,
    (170,90,50),
    (179,255,255),
)
mask = cv2.bitwise_or(mask_low,mask_high)
