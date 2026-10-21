#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("check_moveit", anonymous=True)

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("manipulator")

group.set_end_effector_link("tool0")
group.set_pose_reference_frame("world")

rospy.sleep(1.0)

print("规划分组：", robot.get_group_names())
print("规划坐标系：", group.get_planning_frame())
print("末端链接：", group.get_end_effector_link())
print("是否包含吸盘：", "vacuum_pad" in robot.get_link_names())

pose = group.get_current_pose("tool0")
print("当前末端位姿：")
print(pose)

moveit_commander.roscpp_shutdown()