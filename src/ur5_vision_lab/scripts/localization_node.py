#!/usr/bin/env python3
"""Transform an image-derived surface point; infer center for a known upright cube."""
import rospy
import tf2_ros
import tf2_geometry_msgs  # Registers PointStamped transforms with tf2.
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class Localization:
    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.frame = rospy.get_param('~world_frame', 'world')
        self.size = float(rospy.get_param('~cube_size', 0.04))
        self.center_pub = rospy.Publisher('object_center_world', PointStamped, queue_size=1)
        self.pose_pub = rospy.Publisher('object_pose_world', PoseStamped, queue_size=1)
        self.surface_pub = rospy.Publisher('object_surface_world', PointStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher('localization_status', String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber('object_surface_camera', PointStamped, self.on_point, queue_size=1)

    def on_point(self, camera_point):
        try:
            surface = self.buffer.transform(camera_point, self.frame, timeout=rospy.Duration(0.1))
            center = PointStamped()
            center.header = surface.header
            center.point.x, center.point.y = surface.point.x, surface.point.y
            # This lab assumes an upright 4 cm cube viewed from above.
            center.point.z = surface.point.z - self.size / 2
            pose = PoseStamped()
            pose.header = center.header
            pose.pose.position = center.point
            pose.pose.orientation.w = 1.0
            self.surface_pub.publish(surface)
            self.center_pub.publish(center)
            self.pose_pub.publish(pose)
            marker = Marker()
            marker.header = center.header
            marker.ns, marker.id, marker.type = 'estimated_cube', 0, Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = marker.scale.y = marker.scale.z = self.size
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 0.8
            marker.lifetime = rospy.Duration(0.6)
            self.marker_pub.publish(MarkerArray(markers=[marker]))
            self.status_pub.publish(String(data='LOCALIZED'))
            rospy.loginfo_throttle(2, 'Estimated center in %s: x=%.3f y=%.3f z=%.3f m' % (
                self.frame, center.point.x, center.point.y, center.point.z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            self.status_pub.publish(String(data='TF_UNAVAILABLE: ' + str(error)))
            rospy.logwarn_throttle(2, 'TF transform failed: ' + str(error))


if __name__ == '__main__':
    rospy.init_node('localization')
    node = Localization()
    rospy.spin()
