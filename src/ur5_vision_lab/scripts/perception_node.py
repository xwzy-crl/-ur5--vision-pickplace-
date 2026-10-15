#!/usr/bin/env python3
"""RGB + depth + intrinsics -> observed cube top-surface point in camera frame."""
import cv2
import message_filters
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from ur5_vision_lab.vision_math import red_region, depth_metres, median_depth, back_project


class Perception:
    def __init__(self):
        self.bridge = CvBridge()
        self.info = None
        self.last_frame = None
        self.point_pub = rospy.Publisher('object_surface_camera', PointStamped, queue_size=1)
        self.debug_pub = rospy.Publisher('debug_image', Image, queue_size=1)
        self.detected_pub = rospy.Publisher('detected', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('perception_status', String, queue_size=1, latch=True)
        self.info_sub = rospy.Subscriber(rospy.get_param('~info_topic'), CameraInfo, self.on_info)
        self.rgb_sub = message_filters.Subscriber(rospy.get_param('~rgb_topic'), Image, queue_size=1, buff_size=2**24)
        self.depth_sub = message_filters.Subscriber(rospy.get_param('~depth_topic'), Image, queue_size=1, buff_size=2**24)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.on_images)
        self.timer = rospy.Timer(rospy.Duration(1), self.watchdog)
        self.status('WAITING_FOR_CAMERA', False)

    def status(self, text, detected):
        self.status_pub.publish(String(data=text))
        self.detected_pub.publish(Bool(data=detected))

    def on_info(self, message):
        self.info = message

    def watchdog(self, event):
        if self.last_frame is None or (rospy.Time.now() - self.last_frame).to_sec() > 2:
            self.status('WAITING_FOR_SYNCHRONIZED_IMAGES', False)
            rospy.logwarn_throttle(5, 'Waiting for synchronized RGB/depth images. Check camera topics and Gazebo pause state.')

    def on_images(self, rgb_message, depth_message):
        self.last_frame = rospy.Time.now()
        if self.info is None:
            self.status('WAITING_FOR_CAMERA_INFO', False)
            return
        info = self.info
        try:
            age = (rospy.Time.now() - rgb_message.header.stamp).to_sec()
            if age > 0.5 or age < -0.1:
                raise ValueError('Stale image timestamp')
            if rgb_message.header.frame_id != depth_message.header.frame_id or info.header.frame_id != rgb_message.header.frame_id:
                raise ValueError('RGB/depth/CameraInfo frame mismatch')
            bgr = self.bridge.imgmsg_to_cv2(rgb_message, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_message, 'passthrough')
            if bgr.shape[:2] != depth.shape or (info.width, info.height) != (rgb_message.width, rgb_message.height):
                raise ValueError('Image/depth/intrinsics resolution mismatch')
            result = red_region(bgr, rospy.get_param('~min_area', 60),
                                rospy.get_param('~saturation_min', 90), rospy.get_param('~value_min', 50))
            debug = bgr.copy()
            if result is None:
                self.status('NO_RED_OBJECT', False)
                cv2.putText(debug, 'No red object', (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                (u, v), region, (x, y, w, h) = result
                z = median_depth(depth_metres(depth, depth_message.encoding), region)
                point = PointStamped()
                point.header = depth_message.header
                point.point.x, point.point.y, point.point.z = back_project(u, v, z, info.K)
                self.point_pub.publish(point)
                self.status('DETECTED', True)
                cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(debug, (int(u), int(v)), 3, (255, 255, 0), -1)
                cv2.putText(debug, 'u=%.1f v=%.1f Z=%.3fm' % (u, v, z), (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                rospy.loginfo_throttle(2, 'Detected red surface: u=%.1f v=%.1f optical Z=%.3f m' % (u, v, z))
            message = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            message.header = rgb_message.header
            self.debug_pub.publish(message)
        except (ValueError, CvBridgeError, cv2.error) as error:
            self.status('INVALID_OBSERVATION: ' + str(error), False)
            rospy.logwarn_throttle(2, str(error))


if __name__ == '__main__':
    rospy.init_node('perception')
    node = Perception()
    rospy.spin()
