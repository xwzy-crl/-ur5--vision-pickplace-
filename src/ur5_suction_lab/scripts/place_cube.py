#!/usr/bin/env python3
import copy
import math
import sys
import time

import moveit_commander
import rospy
from gazebo_msgs.srv import GetLinkState, GetModelState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from lift_cube import apply_matrix, distance, permit_support_contact, positions
from transfer_cube import execute_cartesian

TABLE_TOP = 0.20
HALF_CUBE = 0.02
RELEASE_GAP = 0.008


def read_cube(get_model):
    state = get_model("vision_lab_cube", "world")
    if not state.success:
        raise RuntimeError("无法读取方块实际状态")
    p, q = state.pose.position, state.pose.orientation
    v, w = state.twist.linear, state.twist.angular
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              v.x, v.y, v.z, w.x, w.y, w.z)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("方块状态包含无效数值")
    norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not 0.9 < norm2 < 1.1:
        raise RuntimeError("方块姿态四元数无效")
    up_z = 1.0 - 2.0 * (q.x*q.x + q.y*q.y) / norm2
    if up_z < math.cos(math.radians(5)):
        raise RuntimeError("本节要求方块保持平放，当前倾斜超过 5 度")
    return state


def wait_suction(expected):
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            message = rospy.wait_for_message(
                "/vision_lab/suction/grasping", Bool, timeout=0.5)
        except rospy.ROSException:
            continue
        if message.data == expected:
            return
    raise RuntimeError("吸盘作用状态未变为 %s" % expected)


def wait_settled(get_model, expected_xy):
    deadline = time.monotonic() + 15.0
    stable_since = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        state = read_cube(get_model)
        p = state.pose.position
        v, w = state.twist.linear, state.twist.angular
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        spin = math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z)
        xy_error = math.hypot(p.x - expected_xy.x, p.y - expected_xy.y)
        good = (abs(p.z - (TABLE_TOP + HALF_CUBE)) <= 0.003
                and xy_error <= 0.015 and speed <= 0.01 and spin <= 0.1)
        now = rospy.Time.now().to_sec()
        if good:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= 0.5:
                return state
        else:
            stable_since = None
        time.sleep(0.1)
    raise RuntimeError("方块未在目标桌面位置连续稳定 0.5 秒")


def main():
    rospy.init_node("place_cube", anonymous=True)
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
        raise RuntimeError("请从搬运成功、方块仍附着的状态开始")
    required = {"lesson_floor", "lesson_table", "lesson_camera"}
    if not required.issubset(set(scene.get_known_object_names())):
        raise RuntimeError("规划场景不完整")
    if "lesson_cube" in scene.get_known_object_names():
        raise RuntimeError("附着方块同时出现在世界障碍物中，请检查现场")

    for service in ("/gazebo/get_link_state", "/gazebo/get_model_state",
                    "/vision_lab/suction/off", "/get_planning_scene",
                    "/apply_planning_scene"):
        rospy.wait_for_service(service, timeout=10)
    get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    suction_off = rospy.ServiceProxy("/vision_lab/suction/off", Empty)
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)

    wait_suction(True)
    pad_before, cube_before = positions(get_link, get_model)
    read_cube(get_model)
    if math.hypot(cube_before.x - 0.55, cube_before.y - 0.30) > 0.02:
        raise RuntimeError("方块尚未到达本节放置区域 x=0.55、y=0.30")
    if distance(pad_before, cube_before) > 0.045 or not 0.30 <= cube_before.z <= 0.45:
        raise RuntimeError("方块高度或吸附距离不符合搬运完成状态")

    # 用 Gazebo 实际高度修正本次下降量；桌面和方块尺寸来自当前实验模型。
    target = copy.deepcopy(group.get_current_pose("tool0").pose)
    desired_cube_z = TABLE_TOP + HALF_CUBE + RELEASE_GAP
    dz = desired_cube_z - cube_before.z
    target.position.z += dz
    rospy.loginfo("计划下降 %.1f mm；方块中心目标高度 %.4f m",
                  -dz * 1000, desired_cube_z)

    original_matrix = permit_support_contact(get_scene, apply_scene)
    restored = False
    try:
        execute_cartesian(robot, group, target, label="放置下降")
        rospy.sleep(0.5)
        pad_down, cube_down = positions(get_link, get_model)
        read_cube(get_model)
        expected_pad = copy.deepcopy(pad_before)
        expected_pad.z += dz
        clearance = cube_down.z - HALF_CUBE - TABLE_TOP
        xy_error = math.hypot(cube_down.x - cube_before.x, cube_down.y - cube_before.y)
        rospy.loginfo("方块底面估计距桌面：%.1f mm", clearance * 1000)
        if (distance(pad_down, expected_pad) > 0.012 or xy_error > 0.015
                or not -0.001 <= clearance <= 0.016
                or distance(pad_down, cube_down) > 0.045):
            raise RuntimeError("释放前位置验证失败，吸盘保持开启")
        wait_suction(True)

        suction_off()
        wait_suction(False)
        rospy.loginfo("吸盘已关闭，等待方块落稳")
        settled = wait_settled(get_model, cube_before)
        rospy.loginfo("方块已落稳，中心高度：%.4f m", settled.pose.position.z)

        # off 是物理释放；下面两步更新 MoveIt 的规划模型。
        scene.remove_attached_object(link="vacuum_pad", name="lesson_cube")
        if scene.get_attached_objects(["lesson_cube"]):
            raise RuntimeError("方块附着模型未解除，停止撤离")
        world_pose = PoseStamped()
        world_pose.header.frame_id = "world"
        world_pose.header.stamp = rospy.Time.now()
        world_pose.pose = copy.deepcopy(settled.pose)
        scene.add_box("lesson_cube", world_pose, size=(0.04, 0.04, 0.04))
        poses = scene.get_object_poses(["lesson_cube"])
        if ("lesson_cube" not in poses
                or distance(poses["lesson_cube"].position, settled.pose.position) > 0.002):
            raise RuntimeError("桌面方块的碰撞模型更新失败，停止撤离")
        apply_matrix(apply_scene, original_matrix)
        restored = True

        pad_retreat_start, _ = positions(get_link, get_model)
        retreat = copy.deepcopy(group.get_current_pose("tool0").pose)
        retreat.position.z += 0.10
        execute_cartesian(robot, group, retreat, label="末端撤离")
        rospy.sleep(0.5)
        final_cube = wait_settled(get_model, settled.pose.position)
        pad_after, _ = positions(get_link, get_model)
        expected_retreat = copy.deepcopy(pad_retreat_start)
        expected_retreat.z += 0.10
        drift = distance(final_cube.pose.position, settled.pose.position)
        gap = pad_after.z - (final_cube.pose.position.z + HALF_CUBE)
        rospy.loginfo("撤离后方块位移：%.1f mm；吸盘距方块顶面：%.1f mm",
                      drift * 1000, gap * 1000)
        if distance(pad_after, expected_retreat) > 0.012 or drift > 0.005 or gap < 0.08:
            raise RuntimeError("撤离后的实际状态验证失败")
        wait_suction(False)
        rospy.loginfo("放置验证通过：方块留在桌面，吸盘已关闭，末端已撤离")
    finally:
        if not restored:
            apply_matrix(apply_scene, original_matrix)


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        main()
    except Exception as error:
        rospy.logerr("放置中止：%s；停止后续动作，请保留现场", error)
        raise
    finally:
        moveit_commander.roscpp_shutdown()
