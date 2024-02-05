import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 初始化cv_bridge
bridge = CvBridge()
cx = 0  # Record center x coordinate


def image_callback(ros_image):
    try:
        # 将ROS的图像消息转换为OpenCV的图像格式
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")  # To BGR Format
    except Exception as e:
        print(e)
        return

    target_contour, processed_frame = process_frame(frame)

    if target_contour is not None:
        cx = draw_contour_with_center(processed_frame, target_contour)
        cx_pub.publish(cx)

    cv2.imshow("Frame", processed_frame)


def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    black_lower = np.array([0, 0, 0])  # 阈值
    black_upper = np.array([179, 80, 230])
    binary_black = cv2.inRange(hsv, black_lower, black_upper)  # 二值化
    kernel = np.ones((5, 5), np.uint8)  # 降噪卷积核
    erosion = cv2.erode(binary_black, kernel, iterations=4)
    dilate = cv2.dilate(erosion, kernel, iterations=4)
    binary_inverted = cv2.bitwise_not(dilate)
    contours, hierarchy = cv2.findContours(
        binary_inverted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )  # 描边
    min_contour_area = 10000

    expected_aspect_ratio = 3.0 / 2.0  # Width宽 to比 height高 ratio
    aspect_tolerance = 1

    # 宽高比筛选框框
    target_contour = None
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            rect = cv2.minAreaRect(contour)
            width, height = rect[1]
            if width == 0 or height == 0:
                continue

            aspect_ratio = height / width if height > width else width / height

            if abs(aspect_ratio - expected_aspect_ratio) < aspect_tolerance:
                target_contour = contour
                break

    return target_contour, frame


# 框框+中心
def draw_contour_with_center(frame, contour):
    # global cx

    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)  # <- ? Probably error
    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
        return cx
    else:
        return -1


cx_pub = rospy.Publisher("fishmonger_cx", Int16, queue_size=10)


def main():
    rospy.init_node("fishmonger", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    rospy.spin()
    # rate = rospy.Rate(10)  # 10hz
    # try:
        # Publish the cx value
        # rospy.loginfo("Fisher start sending")
        # cx_pub.publish(cx)
        # rospy.spin()
        # rate.sleep()
    # except KeyboardInterrupt:
        # print("Shutting down")


if __name__ == "__main__":
    main()
