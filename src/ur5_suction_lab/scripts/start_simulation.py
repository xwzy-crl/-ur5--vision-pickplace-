#!/usr/bin/env python3

import math
import time
import rospy

from geometry_msgs.msg import Vector3
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from std_srvs.srv import Empty


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


rospy.init_node("start_simulation", anonymous=True)

services = [
    "/gazebo/get_physics_properties",
    "/gazebo/set_physics_properties",
    "/gazebo/pause_physics",
    "/gazebo/unpause_physics",
]

for name in services:
    rospy.wait_for_service(name, timeout=30.0)

get_physics = rospy.ServiceProxy(
    services[0], GetPhysicsProperties
)
set_physics = rospy.ServiceProxy(
    services[1], SetPhysicsProperties
)
pause = rospy.ServiceProxy(services[2], Empty)
unpause = rospy.ServiceProxy(services[3], Empty)

original = get_physics()

if not original.success:
    raise RuntimeError(original.status_message)

if not original.pause:
    raise RuntimeError("请先用 paused=true 重新启动场景")

if original.gravity.z >= -0.1:
    raise RuntimeError("请重新启动场景，确保原始重力正常")


def change_gravity(gravity):
    result = set_physics(
        original.time_step,
        original.max_update_rate,
        gravity,
        original.ode_config
    )
    if not result.success:
        raise RuntimeError(result.status_message)


try:
    change_gravity(Vector3(0.0, 0.0, 0.0))
    unpause()

    topic = "/eff_joint_traj_controller/state"

    rospy.wait_for_message(
        topic, JointTrajectoryControllerState, timeout=30.0
    )

    time.sleep(1.0)

    state = rospy.wait_for_message(
        topic, JointTrajectoryControllerState, timeout=10.0
    )

    expected = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.57,
        "elbow_joint": 0.0,
        "wrist_1_joint": -1.57,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }
    deadline = time.monotonic() + 30.0
    stable_since = None

    while True:
        state = rospy.wait_for_message(
            topic,
            JointTrajectoryControllerState,
            timeout=5.0
        )

        measurements = []

        for name, target in expected.items():
            i = state.joint_names.index(name)

            desired = state.desired.positions[i]
            actual = state.actual.positions[i]
            velocity = state.actual.velocities[i]

            if not all(math.isfinite(v) for v in
                       (desired, actual, velocity)):
                raise RuntimeError(name + " 出现无效数值")

            target_error = abs(
                angle_difference(desired, target)
            )
            tracking_error = abs(
                angle_difference(actual, desired)
            )

            if target_error > 0.1:
                raise RuntimeError(
                    "%s 初始目标错误：期望 %.3f，收到 %.3f rad"
                    % (name, target, desired)
                )

            measurements.append(
                (name, abs(velocity), tracking_error)
            )

        fastest = max(measurements, key=lambda item: item[1])
        worst = max(measurements, key=lambda item: item[2])

        report = (
            "最大速度：%s = %.4f rad/s；"
            "最大位置误差：%s = %.4f rad"
            % (fastest[0], fastest[1], worst[0], worst[2])
        )

        rospy.loginfo_throttle(1.0, report)

        if fastest[1] <= 0.05 and worst[2] <= 0.03:
            stamp = state.header.stamp.to_sec()

            if stable_since is None:
                stable_since = stamp

            if stamp - stable_since >= 1.0:
                break
        else:
            stable_since = None

        if time.monotonic() >= deadline:
            raise RuntimeError("30秒内未稳定；" + report)

        time.sleep(0.05)

    change_gravity(original.gravity)
    rospy.loginfo("初始状态检查通过，已恢复正常重力")

except Exception:
    pause()
    rospy.logerr("启动检查失败，仿真已暂停；请保留错误输出")
    raise
