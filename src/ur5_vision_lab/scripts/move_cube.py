#!/usr/bin/env python3
"""Move the test object. This service is not used by perception/localization."""
import argparse
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('move_lab_cube', anonymous=True)
    cx = rospy.get_param('/vision_lab/scene/table_x', 0.45)
    cy = rospy.get_param('/vision_lab/scene/table_y', 0.30)
    if abs(args.x-cx) > 0.30 or abs(args.y-cy) > 0.20:
        raise SystemExit('Requested position is outside the usable table area.')
    rospy.wait_for_service('/gazebo/set_model_state', timeout=10)
    state = ModelState()
    state.model_name, state.reference_frame = 'vision_lab_cube', 'world'
    state.pose.position.x, state.pose.position.y, state.pose.position.z = args.x, args.y, 0.225
    state.pose.orientation.w = 1.0
    response = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(model_state=state)
    if not response.success:
        raise SystemExit(response.status_message)
    print('Moved test cube. Watch the CAMERA-DERIVED position update after settling.')
