#!/usr/bin/env python3
"""Sequentially add the table, camera and cube to an existing ROS Gazebo world."""
from pathlib import Path
import math
import rospy
from gazebo_msgs.srv import SpawnModel, GetWorldProperties
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


def main():
    rospy.init_node('scene')
    model_dir = Path(rospy.get_param('~model_dir'))
    x = float(rospy.get_param('~table_x', 0.45))
    y = float(rospy.get_param('~table_y', 0.30))
    camera_x = float(rospy.get_param('~camera_x', x))
    camera_y = float(rospy.get_param('~camera_y', y))
    camera_z = float(rospy.get_param('~camera_z', 1.2))
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=60)
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=15)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    if not properties.success:
        raise RuntimeError(properties.status_message)
    existing = set(properties.model_names)
    entries = [('table', x, y, 0.10), ('camera', camera_x, camera_y, camera_z),
               ('cube', x, y, 0.225)]
    for name, px, py, pz in entries:
        model_name = 'vision_lab_' + name
        if model_name in existing:
            rospy.logwarn('%s exists; reusing it. Restart Gazebo if model/camera settings changed.', model_name)
            continue
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = px, py, pz
        pose.orientation.w = 1.0
        if name == 'camera':
            pose.orientation.y = math.sin(math.pi / 4)
            pose.orientation.w = math.cos(math.pi / 4)
        result = spawn(model_name=model_name, model_xml=(model_dir / (name + '.sdf')).read_text(),
                       robot_namespace='/', initial_pose=pose, reference_frame='world')
        if not result.success:
            raise RuntimeError(result.status_message)
        rospy.loginfo('Spawned %s', model_name)
    publisher = rospy.Publisher('scene_marker', Marker, queue_size=1, latch=True)
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.ns, marker.id, marker.type = 'table', 0, Marker.CUBE
    marker.pose.orientation.w = 1.0
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.1
    marker.scale.x, marker.scale.y, marker.scale.z = 0.7, 0.5, 0.2
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.5, 0.5, 0.5, 0.3
    publisher.publish(marker)
    rospy.loginfo('Scene ready. Table top z=0.20 m; cube side=0.04 m.')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
