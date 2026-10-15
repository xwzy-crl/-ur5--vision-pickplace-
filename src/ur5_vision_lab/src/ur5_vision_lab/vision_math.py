"""ROS-independent helpers. Depth is optical-axis distance, in metres."""

import numpy as np


def red_region(bgr, min_area=60, saturation_min=90, value_min=50):
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, (0, saturation_min, value_min), (10, 255, 255))
    high = cv2.inRange(hsv, (170, saturation_min, value_min), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < min_area:
        return None
    moments = cv2.moments(contour)
    if moments['m00'] <= 0:
        return None
    u, v = moments['m10'] / moments['m00'], moments['m01'] / moments['m00']
    region = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(region, [contour], -1, 255, -1)
    region = cv2.erode(region, np.ones((3, 3), np.uint8), iterations=1)
    return (u, v), region, cv2.boundingRect(contour)


def depth_metres(depth, encoding):
    if encoding == '32FC1':
        return depth.astype(np.float64)
    if encoding == '16UC1':
        return depth.astype(np.float64) / 1000.0
    raise ValueError('Unsupported depth encoding: ' + encoding)


def median_depth(depth, region, near=0.1, far=3.0):
    values = depth[region > 0]
    valid = values[np.isfinite(values) & (values > near) & (values < far)]
    if len(values) < 5 or len(valid) < max(5, int(len(values) * 0.5)):
        raise ValueError('Too few valid depth pixels inside the object')
    return float(np.median(valid))


def back_project(u, v, z, k):
    k = np.asarray(k, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    if not np.isfinite(z) or z <= 0:
        raise ValueError('Invalid depth')
    return ((u - k[0, 2]) * z / k[0, 0],
            (v - k[1, 2]) * z / k[1, 1], z)
