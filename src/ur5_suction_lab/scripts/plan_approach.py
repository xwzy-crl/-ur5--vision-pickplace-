#!/usr/bin/env python3

import math
import sys
from gazebo_msgs.srv import GetLinkState

import rospy
import moveit_commander

from geometry_msgs.msg import PointStamped, PoseStamped


def add_box(scene, name, position, size):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.orientation.w = 1.0

    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]

    scene.add_box(name, pose, size=size)


def main():
    rospy.init_node("plan_approach", anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface(
        synchronous=True
    )

    group.set_end_effector_link("tool0")
    group.set_pose_reference_frame("world")
    group.set_planning_time(15.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(0.08)
    group.set_max_acceleration_scaling_factor(0.08)

    rospy.sleep(1.0)

    message = rospy.wait_for_message(
        "/vision_lab/object_center_world",
        PointStamped,
        timeout=15.0
    )

    if message.header.frame_id != "world":
        raise RuntimeError("视觉坐标系不是 world")

    age = (rospy.Time.now() - message.header.stamp).to_sec()

    if not 0.0 <= age <= 0.6:
        raise RuntimeError("视觉数据时间异常或已经过期")

    x = message.point.x
    y = message.point.y
    z = message.point.z

    if not all(math.isfinite(v) for v in (x, y, z)):
        raise RuntimeError("视觉坐标包含无效数值")

    if not (0.15 <= x <= 0.75
            and 0.10 <= y <= 0.50
            and 0.20 <= z <= 0.24):
        raise RuntimeError("目标超出本实验的桌面检查范围")

    add_box(
        scene, "lesson_floor",
        (0.0, 0.0, -0.025),
        (4.0, 4.0, 0.05)
    )

    add_box(
        scene, "lesson_table",
        (0.45, 0.30, 0.10),
        (0.70, 0.50, 0.20)
    )

    add_box(
        scene, "lesson_cube",
        (x, y, z),
        (0.04, 0.04, 0.04)
    )

    add_box(
        scene, "lesson_camera",
        (0.45, 0.30, 1.20),
        (0.08, 0.08, 0.08)
    )

    required = {
        "lesson_floor", "lesson_table",
        "lesson_cube", "lesson_camera"
    }

    if not required.issubset(scene.get_known_object_names()):
        raise RuntimeError("规划场景中的障碍物尚未添加完整")

    cube_size = 0.04
    tool_length = 0.08
    gap = 0.004
    clearance = 0.12

    target = PoseStamped()
    target.header.frame_id = "world"

    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = (
        z + cube_size / 2.0
        + gap + clearance + tool_length
    )

    target.pose.orientation.x = 1.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    rospy.loginfo(
        "tool0 接近目标：x=%.3f, y=%.3f, z=%.3f m",
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z
    )

    group.set_start_state_to_current_state()
    group.set_pose_target(target, "tool0")

    result = group.plan()

    if isinstance(result, tuple):
        success = result[0]
        trajectory = result[1]
    else:
        trajectory = result
        success = bool(trajectory.joint_trajectory.points)

    group.clear_pose_targets()

    points = trajectory.joint_trajectory.points

    if not success or len(points) < 2:
        raise RuntimeError("接近目标的轨迹规划失败")

    rospy.loginfo(
        "规划成功：%d 个轨迹点，预计运动时间 %.2f 秒",
        len(points),
        points[-1].time_from_start.to_sec()
    )

    if "--execute" not in rospy.myargv():
        rospy.loginfo("规划完成；添加 --execute 参数可执行")
        return

    # 执行前重新确认方块没有明显移动
    latest = rospy.wait_for_message(
        "/vision_lab/object_center_world",
        PointStamped,
        timeout=5.0
    )

    latest_age = (
        rospy.Time.now() - latest.header.stamp
    ).to_sec()

    values = (
        latest.point.x,
        latest.point.y,
        latest.point.z
    )

    if not all(math.isfinite(v) for v in values):
        raise RuntimeError("最新视觉坐标无效")

    if latest.header.frame_id != "world":
        raise RuntimeError("最新视觉坐标系异常")

    if not 0.0 <= latest_age <= 0.6:
        raise RuntimeError("最新视觉数据时间异常")

    shift = math.sqrt(
        (values[0] - x) ** 2
        + (values[1] - y) ** 2
        + (values[2] - z) ** 2
    )

    if shift > 0.01:
        raise RuntimeError("方块已移动超过 1 厘米，请重新规划")

    rospy.loginfo("开始执行接近轨迹")

    try:
        executed = group.execute(trajectory, wait=True)
    finally:
        group.stop()

    if not executed:
        raise RuntimeError("MoveIt 报告轨迹执行失败")

    rospy.sleep(0.5)

    # 查询 Gazebo 中实际吸盘的位置和姿态
    rospy.wait_for_service(
        "/gazebo/get_link_state",
        timeout=5.0
    )

    get_link = rospy.ServiceProxy(
        "/gazebo/get_link_state",
        GetLinkState
    )

    response = get_link("robot::vacuum_pad", "world")

    if not response.success:
        raise RuntimeError(response.status_message)

    position = response.link_state.pose.position
    expected_z = target.pose.position.z - tool_length

    position_error = math.sqrt(
        (position.x - target.pose.position.x) ** 2
        + (position.y - target.pose.position.y) ** 2
        + (position.z - expected_z) ** 2
    )

    q = response.link_state.pose.orientation
    q_norm = math.sqrt(
        q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2
    )

    if not math.isfinite(q_norm) or q_norm < 1e-9:
        raise RuntimeError("实际吸盘姿态无效")

    angle_error = 2.0 * math.acos(
        min(1.0, abs(q.x) / q_norm)
    )

    rospy.loginfo(
        "实际吸盘位置：(%.4f, %.4f, %.4f) m",
        position.x, position.y, position.z
    )

    rospy.loginfo(
        "位置误差：%.2f mm；姿态误差：%.2f 度",
        position_error * 1000.0,
        math.degrees(angle_error)
    )

    if (not math.isfinite(position_error)
            or position_error > 0.008
            or angle_error > math.radians(3.0)):
        raise RuntimeError("实际工具位姿未达到本次验收要求")

    rospy.loginfo("接近动作完成，实际吸盘位姿验证通过")


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)

    try:
        main()
    finally:
        moveit_commander.roscpp_shutdown()