#!/usr/bin/env python3
import copy
import inspect
import math
import sys

import moveit_commander
import rospy
from gazebo_msgs.srv import GetLinkState, GetModelState
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPlanningSceneRequest
from std_msgs.msg import Bool


def distance(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)


def positions(get_link, get_model):
    pad = get_link("robot::vacuum_pad", "world")
    cube = get_model("vision_lab_cube", "world")
    if not pad.success or not cube.success:
        raise RuntimeError("无法读取吸盘或方块的位置")
    p = pad.link_state.pose.position
    c = cube.pose.position
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, c.x, c.y, c.z)):
        raise RuntimeError("Gazebo 返回了无效坐标")
    return p, c


def apply_matrix(apply_scene, matrix):
    update = PlanningScene()
    update.is_diff = True
    update.robot_state.is_diff = True
    update.allowed_collision_matrix = matrix
    if not apply_scene(update).success:
        raise RuntimeError("碰撞规则更新失败")


def permit_support_contact(get_scene, apply_scene):
    request = GetPlanningSceneRequest()
    request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    original = get_scene(request).scene.allowed_collision_matrix
    matrix = copy.deepcopy(original)

    # 本实验的方块和桌面没有默认碰撞豁免规则。
    pair = ("lesson_cube", "lesson_table")
    if any(name in matrix.default_entry_names for name in pair):
        raise RuntimeError("存在额外默认碰撞规则，请先检查规划场景")
    for name in pair:
        if name not in matrix.entry_names:
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_names.append(name)
            row = AllowedCollisionEntry()
            row.enabled = [False] * len(matrix.entry_names)
            matrix.entry_values.append(row)

    i, j = [matrix.entry_names.index(name) for name in pair]
    matrix.entry_values[i].enabled[j] = True
    matrix.entry_values[j].enabled[i] = True
    apply_matrix(apply_scene, matrix)
    return original


def main():
    rospy.init_node("lift_cube", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    group.set_end_effector_link("tool0")
    group.set_pose_reference_frame("world")
    if group.get_planning_frame() != "world":
        raise RuntimeError("规划坐标系必须为 world")
    rospy.sleep(1.0)

    if scene.get_attached_objects(["lesson_cube"]):
        raise RuntimeError("方块已经处于附着状态，请勿重复执行本节")
    required = {"lesson_floor", "lesson_table", "lesson_cube", "lesson_camera"}
    if not required.issubset(set(scene.get_known_object_names())):
        raise RuntimeError("规划场景不完整，请保留接近和下降实验的 MoveIt 进程")

    for service in ("/gazebo/get_link_state", "/gazebo/get_model_state",
                    "/get_planning_scene", "/apply_planning_scene"):
        rospy.wait_for_service(service, timeout=10.0)
    get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)

    if not rospy.wait_for_message("/vision_lab/suction/grasping", Bool, timeout=3).data:
        raise RuntimeError("吸盘尚未作用于物体，请先检查下降和吸附结果")
    pad_before, cube_before = positions(get_link, get_model)
    if distance(pad_before, cube_before) > 0.045:
        raise RuntimeError("方块中心距吸盘过远，停止抬升")
    rospy.loginfo("抬升前：方块中心高度 %.4f m", cube_before.z)

    target = copy.deepcopy(group.get_current_pose("tool0").pose)
    target.position.z += 0.12

    original_matrix = permit_support_contact(get_scene, apply_scene)
    try:
        # 仅改变 MoveIt 的碰撞模型；Gazebo 中的吸附仍由吸盘插件实现。
        scene.attach_box("vacuum_pad", "lesson_cube",
                         touch_links=["vacuum_pad"])
        if "lesson_cube" not in scene.get_attached_objects(["lesson_cube"]):
            raise RuntimeError("方块附着模型创建失败")
        if "lesson_cube" in scene.get_known_object_names():
            raise RuntimeError("方块仍被登记为桌上的障碍物")

        group.set_start_state_to_current_state()
        options = {"avoid_collisions": True}
        if "jump_threshold" in inspect.signature(group.compute_cartesian_path).parameters:
            options["jump_threshold"] = 0.0
        path, fraction = group.compute_cartesian_path(
            [target], eef_step=0.003, **options)
        rospy.loginfo("抬升路径完成比例：%.2f%%", fraction * 100.0)
        if not math.isfinite(fraction) or fraction < 1.0 - 1e-6:
            raise RuntimeError("抬升路径不完整，停止执行")

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
        times = [point.time_from_start.to_sec() for point in path.joint_trajectory.points]
        if (len(times) < 2 or not all(math.isfinite(t) and t >= 0 for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise RuntimeError("轨迹时间参数无效")

        rospy.loginfo("开始执行 12 厘米抬升")
        try:
            executed = group.execute(path, wait=True)
        finally:
            group.stop()
        if not executed:
            raise RuntimeError("控制器未成功执行抬升")

        rospy.sleep(0.5)
        pad_after, cube_after = positions(get_link, get_model)
        pad_rise = pad_after.z - pad_before.z
        cube_rise = cube_after.z - cube_before.z
        separation = distance(pad_after, cube_after)
        rospy.loginfo("吸盘实际上升：%.1f mm", pad_rise * 1000)
        rospy.loginfo("方块实际上升：%.1f mm", cube_rise * 1000)
        rospy.loginfo("方块中心距吸盘：%.1f mm", separation * 1000)

        expected_pad = copy.deepcopy(pad_before)
        expected_pad.z += 0.12
        if distance(pad_after, expected_pad) > 0.012:
            raise RuntimeError("吸盘未到达预期抬升位置")
        if abs(cube_rise - 0.12) > 0.02 or separation > 0.045:
            raise RuntimeError("未通过方块随动验证，请保留现场和日志")
        if not rospy.wait_for_message("/vision_lab/suction/grasping", Bool, timeout=3).data:
            raise RuntimeError("抬升后吸盘作用状态为 False")
        rospy.loginfo("抬升验证通过；保持吸盘开启和方块附着，等待搬运")
    finally:
        # 本次运行期间不要启动其他修改碰撞规则的程序。
        apply_matrix(apply_scene, original_matrix)


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        main()
    except Exception as error:
        rospy.logerr("抬升中止：%s；保留吸附状态，请检查现场", error)
        raise
    finally:
        moveit_commander.roscpp_shutdown()
