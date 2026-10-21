#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped


def on_target(message):
    frame = message.header.frame_id

    if frame !="world":
        rospy.logwarn_throttle(1.0,"目标坐标系不是world")
        return

    x=message.point.x
    y=message.point.y
    z = message.point.z

    cube_size = 0.04
    approach_height = 0.12

    top_z = z + cube_size / 2.0
    approach_z = top_z + approach_height

    rospy.loginfo_throttle(
        1.0,
        "中心=(%.3f, %.3f, %.3f) m；"
        "顶面高度=%.3f m；"
        "接近参考点=(%.3f, %.3f, %.3f) m"
        % (x, y, z, top_z, x, y, approach_z)
    )