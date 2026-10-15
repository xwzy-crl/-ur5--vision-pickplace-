#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def on_status(message):
    # 从收到的 ROS 消息中取出字符串
    text = message.data

    # 在接收节点的日志中显示
    rospy.loginfo("接收节点收到：%s", text)


def main():
    rospy.init_node("status_listener")

    # 注册订阅：收到消息后，调用 on_status
    rospy.Subscriber(
        "lesson_status",
        String,
        on_status,
        queue_size=10
    )

    # 保持节点运行，等待消息
    rospy.spin()


if __name__ == "__main__":
    main()