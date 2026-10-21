#!/usr/bin/env python3
import copy
import inspect
import math
import sys

import moveit_commander
import rospy
from gazebo_msgs.srv import GetLinkState, GetModelState
from std_msgs.msg import Bool

# 两个脚本放在同一个 scripts 目录，复用上一节的位置读取函数。
from lift_cube import distance, positions


def execute_cartesian(robot, group, target, label="搬运"):
    group.set_start_state_to_current_state()
    options = {"avoid_collisions": True}
    if "jump_threshold" in inspect.signature(group.compute_cartesian_path).parameters:
        options["jump_threshold"] = 0.0
    path, fraction = group.compute_cartesian_path(
        [target], eef_step=0.003, **options)
    rospy.loginfo("%s路径完成比例：%.2f%%", label, fraction * 100)
    if not math.isfinite(fraction) or fraction < 1.0 - 1e-6:
        raise RuntimeError("路径不完整，停止执行")

    points = path.joint_trajectory.points
    if len(points) < 2:
        raise RuntimeError("轨迹点不足")
    for previous, current in zip(points, points[1:]):
        jumps = [abs(a - b) for a, b in zip(current.positions, previous.positions)]
        if not jumps or not all(math.isfinite(v) for v in jumps) or max(jumps) > 0.35:
            raise RuntimeError("轨迹包含无效关节值或明显跳变")

    path = group.retime_trajectory(
        robot.get_current_state(), path,
        velocity_scaling_factor=0.05, acceleration_scaling_factor=0.05)
    times = [p.time_from_start.to_sec() for p in path.joint_trajectory.points]
    if (len(times) < 2 or not all(math.isfinite(t) and t >= 0 for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise RuntimeError("轨迹时间参数无效")

    rospy.loginfo("开始执行%s", label)
    try:
        executed = group.execute(path, wait=True)
    finally:
        group.stop()
    if not executed:
        raise RuntimeError("控制器未成功执行轨迹")


def main():
    rospy.init_node("transfer_cube", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    group.set_end_effector_link("tool0")
    group.set_pose_reference_frame("world")
    if group.get_planning_frame() != "world":
        raise RuntimeError("规划坐标系必须为 world")
    rospy.sleep(1.0)

    attached = scene.get_attached_objects(["lesson_cube"])
    if "lesson_cube" not in attached or attached["lesson_cube"].link_name != "vacuum_pad":
        raise RuntimeError("请先完成抬升，保留方块在 vacuum_pad 上的附着模型")
    required = {"lesson_floor", "lesson_table", "lesson_camera"}
    if not required.issubset(set(scene.get_known_object_names())):
        raise RuntimeError("规划场景不完整")
    if "lesson_cube" in scene.get_known_object_names():
        raise RuntimeError("方块仍被登记为桌上的障碍物")
    if not rospy.wait_for_message("/vision_lab/suction/grasping", Bool, timeout=3).data:
        raise RuntimeError("吸盘作用状态为 False")

    rospy.wait_for_service("/gazebo/get_link_state", timeout=10)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    pad_before, cube_before = positions(get_link, get_model)
    if distance(pad_before, cube_before) > 0.045 or cube_before.z < 0.30:
        raise RuntimeError("方块距离吸盘过远，或尚未抬升到搬运高度")

    current = group.get_current_pose("tool0").pose
    target = copy.deepcopy(current)
    target.position.x = 0.55
    dx = target.position.x - current.position.x
    if not math.isfinite(dx) or abs(dx) > 0.15:
        raise RuntimeError("当前距离目标过远，请检查机械臂位置")
    if abs(dx) < 0.01:
        rospy.loginfo("末端已接近 x=0.55 m，保持当前位置")
        return
    rospy.loginfo("目标 tool0：x=%.3f，y=%.3f，z=%.3f m",
                  target.position.x, target.position.y, target.position.z)
    execute_cartesian(robot, group, target)

    rospy.sleep(0.5)
    pad_after, cube_after = positions(get_link, get_model)
    expected_pad = copy.deepcopy(pad_before)
    expected_cube = copy.deepcopy(cube_before)
    expected_pad.x += dx
    expected_cube.x += dx
    pad_error = distance(pad_after, expected_pad)
    cube_error = distance(cube_after, expected_cube)
    separation = distance(pad_after, cube_after)
    rospy.loginfo("方块实际 X 位移：%.1f mm", (cube_after.x - cube_before.x) * 1000)
    rospy.loginfo("方块高度变化：%.1f mm", (cube_after.z - cube_before.z) * 1000)
    rospy.loginfo("吸盘位置误差：%.1f mm；方块位置误差：%.1f mm",
                  pad_error * 1000, cube_error * 1000)
    rospy.loginfo("方块中心距吸盘：%.1f mm", separation * 1000)
    if pad_error > 0.012 or cube_error > 0.015 or separation > 0.045:
        raise RuntimeError("搬运随动验证失败，请保留现场")
    if not rospy.wait_for_message("/vision_lab/suction/grasping", Bool, timeout=3).data:
        raise RuntimeError("搬运后吸盘作用状态为 False")
    rospy.loginfo("搬运验证通过；保持吸盘开启，等待下降放置")


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        main()
    except Exception as error:
        rospy.logerr("搬运中止：%s；保留吸附状态，请检查现场", error)
        raise
    finally:
        moveit_commander.roscpp_shutdown()
