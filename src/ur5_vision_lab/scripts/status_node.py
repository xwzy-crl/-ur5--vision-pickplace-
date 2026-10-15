#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def main():
    # 注册为 ROS 节点，launch 可以覆盖这里的名称
    rospy.init_node("status")

    # ~ 表示读取这个节点自己的私有参数
    frequency = float(rospy.get_param("~publish_rate", 1.0))

    if frequency <= 0:
        rospy.logerr("publish_rate 必须大于 0")
        return

    # 创建发布器：发布字符串消息
    publisher = rospy.Publisher(
        "lesson_status", String, queue_size=10
    )

    rate = rospy.Rate(frequency)
    count = 0

    while not rospy.is_shutdown():
        # 每轮读取一次课程名称
        lesson = rospy.get_param("lesson_name", "unset")

        count += 1
        text = f"lesson={lesson}; count={count}"

        publisher.publish(String(data=text))
        rospy.loginfo(text)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass