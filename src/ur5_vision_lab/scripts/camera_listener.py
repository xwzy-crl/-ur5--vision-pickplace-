#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()


def on_image(message, debug_publisher):
    try:
        # 创建可供绘制的独立图像数组。
        image = bridge.imgmsg_to_cv2(
            message,
            desired_encoding="bgr8"
        ).copy()
    except CvBridgeError as error:
        rospy.logerr("图像转换失败：%s", error)
        return

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask_low = cv2.inRange(hsv, (0, 90, 50), (10, 255, 255))
    mask_high = cv2.inRange(hsv, (170, 90, 50), (179, 255, 255))
    mask = cv2.bitwise_or(mask_low, mask_high)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    detected = False

    if contours:
        target = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(target)
        moments = cv2.moments(target)

        if area >= 60 and moments["m00"] > 0:
            detected = True

            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])

            # 获取包围目标的矩形：左上角坐标、宽度、高度。
            x, y, width, height = cv2.boundingRect(target)

            # 画绿色矩形。
            cv2.rectangle(
                image,
                (x, y),
                (x + width - 1, y + height - 1),
                (0, 255, 0),
                2
            )

            # 画黄色中心点，-1 表示填充圆形。
            cv2.circle(image, (u, v), 4, (0, 255, 255), -1)

            cv2.putText(
                image,
                f"u={u} v={v}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

            rospy.loginfo_throttle(
                1.0,
                f"红色目标：u={u}, v={v}"
            )

    if not detected:
        cv2.putText(
            image,
            "No red target",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )
        rospy.loginfo_throttle(1.0, "未检测到有效红色目标")

    # 无论是否检测到目标，都发布当前画面。
    try:
        output_message = bridge.cv2_to_imgmsg(
            image,
            encoding="bgr8"
        )

        # 保留原图像的采集时间和摄像头坐标系。
        output_message.header = message.header

        debug_publisher.publish(output_message)
    except CvBridgeError as error:
        rospy.logerr("输出图像转换失败：%s", error)


def main():
    rospy.init_node("camera_listener")

    topic = rospy.get_param(
        "~image_topic",
        "/camera/color/image_raw"
    )

    debug_publisher = rospy.Publisher(
        "lesson_debug_image",
        Image,
        queue_size=1
    )

    rospy.Subscriber(
        topic,
        Image,
        on_image,
        callback_args=debug_publisher,
        queue_size=1,
        buff_size=4 * 1024 * 1024
    )

    rospy.loginfo("正在订阅：%s", topic)
    rospy.spin()


if __name__ == "__main__":
    main()