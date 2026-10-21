#!/usr/bin/env python3

import copy
import inspect
import math
import sys
import time

import rospy
import moveit_commander

from gazebo_msgs.srv import GetLinkState
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def main():
    rospy.init_node("suction_contact", anonymous=True)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface(
        synchronous=True
    )

    group.set_end_effector_link("tool0")
    group.set_pose_reference_frame("world")

    if group.get_planning_frame() != "world":
        raise RuntimeError("规划坐标系不是 world")

    rospy.sleep(1.0)

    required = {
        "lesson_floor", "lesson_table",
        "lesson_cube", "lesson_camera"
    }

    if not required.issubset(scene.get_known_object_names()):
        raise RuntimeError("上一步的规划场景不完整")

    poses = scene.get_object_poses(["lesson_cube"])

    if "lesson_cube" not in poses:
        raise RuntimeError("没有找到缓存的方块位姿")

    center = poses["lesson_cube"].position

    if not all(math.isfinite(v) for v in
               (center.x, center.y, center.z)):
        raise RuntimeError("缓存坐标无效")

    for service in (
        "/gazebo/get_link_state",
        "/vision_lab/suction/on",
        "/vision_lab/suction/off"
    ):
        rospy.wait_for_service(service, timeout=10.0)

    get_link = rospy.ServiceProxy(
        "/gazebo/get_link_state", GetLinkState
    )
    suction_on = rospy.ServiceProxy(
        "/vision_lab/suction/on", Empty
    )
    suction_off = rospy.ServiceProxy(
        "/vision_lab/suction/off", Empty
    )

    response = get_link("robot::vacuum_pad", "world")

    if not response.success:
        raise RuntimeError(response.status_message)

    pad = response.link_state.pose
    xy_error = math.hypot(
        pad.position.x - center.x,
        pad.position.y - center.y
    )

    if xy_error > 0.01:
        raise RuntimeError("吸盘尚未对准方块上方")

    if not center.z + 0.005 <= pad.position.z <= center.z + 0.25:
        raise RuntimeError("吸盘高度不符合本次下降条件")

    q = pad.orientation
    norm = math.sqrt(
        q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2
    )

    if not math.isfinite(norm) or norm < 1e-9:
        raise RuntimeError("吸盘姿态无效")

    angle = 2.0 * math.acos(min(1.0, abs(q.x) / norm))

    if angle > math.radians(3.0):
        raise RuntimeError("吸盘朝向偏差超过 3 度")

    suction_off()

    # 保持当前 x、y 和姿态，只改变高度
    target = copy.deepcopy(
        group.get_current_pose("tool0").pose
    )

    cube_top = center.z + 0.02
    target.position.z = cube_top + 0.004 + 0.08

    group.set_start_state_to_current_state()

    options = {"avoid_collisions": True}

    parameters = inspect.signature(
        group.compute_cartesian_path
    ).parameters

    if "jump_threshold" in parameters:
        options["jump_threshold"] = 0.0

    path, fraction = group.compute_cartesian_path(
        [target],
        eef_step=0.003,
        **options
    )

    rospy.loginfo("下降路径完成比例：%.2f%%", fraction * 100.0)

    if not math.isfinite(fraction) or fraction < 1.0 - 1e-6:
        raise RuntimeError("下降路径不完整，停止执行")

    points = path.joint_trajectory.points

    if len(points) < 2:
        raise RuntimeError("下降轨迹点不足")

    for before, after in zip(points, points[1:]):
        jump = max(
            abs(a - b)
            for a, b in zip(after.positions, before.positions)
        )
        if jump > 0.35:
            raise RuntimeError("下降轨迹存在较大的关节跳变")

    path = group.retime_trajectory(
        robot.get_current_state(),
        path,
        velocity_scaling_factor=0.05,
        acceleration_scaling_factor=0.05
    )

    times = [
        point.time_from_start.to_sec()
        for point in path.joint_trajectory.points
    ]

    if (len(times) < 2
            or not all(math.isfinite(t) and t >= 0 for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise RuntimeError("下降轨迹时间参数无效")

    rospy.loginfo("开始执行下降")

    try:
        executed = group.execute(path, wait=True)
    finally:
        group.stop()

    if not executed:
        raise RuntimeError("下降执行失败")

    rospy.sleep(0.5)

    response = get_link("robot::vacuum_pad", "world")

    if not response.success:
        raise RuntimeError(response.status_message)

    actual = response.link_state.pose.position

    rospy.loginfo(
        "实际吸盘高度：%.4f m；估计方块顶面：%.4f m",
        actual.z, cube_top
    )

    if abs(actual.z - (cube_top + 0.004)) > 0.01:
        raise RuntimeError("实际下降高度偏差超过 1 厘米")

    if math.hypot(actual.x - center.x, actual.y - center.y) > 0.01:
        raise RuntimeError("下降后吸盘水平偏差超过 1 厘米")

    suction_on()

    detected = False
    deadline = time.monotonic() + 3.0

    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            status = rospy.wait_for_message(
                "/vision_lab/suction/grasping",
                Bool,
                timeout=0.5
            )
        except rospy.ROSException:
            continue

        if status.data:
            detected = True
            break

    if not detected:
        suction_off()
        raise RuntimeError("没有检测到吸盘近距离作用对象")

    rospy.loginfo("吸盘作用状态为 True，保持开启，等待抬升验证")


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)

    try:
        main()
    finally:
        moveit_commander.roscpp_shutdown()