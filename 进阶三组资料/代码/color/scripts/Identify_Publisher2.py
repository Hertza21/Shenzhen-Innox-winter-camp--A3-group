#!/usr/bin/env python
# coding:utf-8
# 初始化
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 初始化cv_bridge
bridge = CvBridge()
# capture = cv2.VideoCapture(7)  # /dev/video7

def image_callback(ros_image):
    try:
        # 将ROS的图像消息转换为OpenCV的图像格式
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")  # To BGR Format
    except Exception as e:
        print(e)
        return
    
    list_color = []
    orange = identify_orange(frame)
    list_color.append(orange)
    red = identify_red(frame)
    list_color.append(red)
    blue = identify_blue(frame)
    list_color.append(blue)
    yellow = identify_yellow(frame)
    list_color.append(yellow)
    green = identify_green(frame)
    list_color.append(green)

    rospy.loginfo("start sending")
    msg = Int16MultiArray()
    msg.data = list_color
    pub.publish(msg)


def sort(box):
    # listx = []
    # listy = []
    # for i in box:
    #     listx.append(i[0])
    #     listy.append(i[1])
    # x1 = min(listx)
    # y1 = min(listy)
    # x2 = max(listx)
    # y2 = max(listy)
    listx = [i[0] for i in box]
    listy = [i[1] for i in box]
    x1, y1, x2, y2 = min(listx), min(listy), max(listx), max(listy)
    # return int((x1+x2)/2) if 50<=abs(x1-x2)<=400 and 50<=abs(y1-y2)<=400 else -1
    if 50 <= abs(x1 - x2) <= 400 and 50 <= abs(y1 - y2) <= 400:
        return int((x1 + x2) / 2)
    else:
        return -1


# def show(frame):  # 设置展示图像函数
#     cv2.namedWindow('image', cv2.WINDOW_NORMAL)
#     cv2.imshow('image', frame)
#     cv2.waitKey(30)


# def read():  # 设置读取视频流函数
#     ret, frame = (
#         capture.read()
#     )  # 摄像头读取,ret为是否成功打开摄像头,true,false。 frame为视频的每一帧图像
#     if ret == True:
#         return frame


def identify_orange(frame):
    len_list = []
    # 图像预处理
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 转换为HSV格式(色调，饱和度，亮度)
    low_orange = np.array([[6, 43, 46]])
    high_orange = np.array([[14, 255, 255]])  # 设定阈值
    mask = cv2.inRange(hsv, low_orange, high_orange)  # 进行掩模运算
    kernal1 = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(mask, kernal1, iterations=4)
    kernal = np.ones((4, 4), np.uint8)
    dilate = cv2.dilate(erosion, kernal, iterations=1)
    img = dilate
    # return(img)
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) != 0:
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        return sort(box)
    else:
        return -1


def identify_red(frame):
    len_list = []
    # 图像预处理
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 转换为HSV格式(色调，饱和度，亮度)
    low_red_0 = np.array([[140, 43, 46]])
    high_red_0 = np.array([[180, 255, 255]])  # 设定阈值
    low_red_1 = np.array([[0, 43, 46]])
    high_red_1 = np.array([[5, 255, 255]])
    mask0 = cv2.inRange(hsv, low_red_0, high_red_0)  # 进行掩模运算
    mask1 = cv2.inRange(hsv, low_red_1, high_red_1)  # 进行掩模运算
    mask = cv2.bitwise_or(mask0, mask1)
    kernal1 = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(mask, kernal1, iterations=4)
    kernal = np.ones((4, 4), np.uint8)
    dilate = cv2.dilate(erosion, kernal, iterations=1)
    img = dilate
    # return(img)
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) != 0:
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        # print(box[0][0],box[0][1],box[3][0],box[3][0])
        return sort(box)
    else:
        return -1


def identify_yellow(frame):
    len_list = []
    # 图像预处理
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 转换为HSV格式(色调，饱和度，亮度)
    low_yellow = np.array([[17, 43, 46]])
    high_yellow = np.array([[35, 255, 255]])  # 设定阈值
    mask = cv2.inRange(hsv, low_yellow, high_yellow)  # 进行掩模运算
    kernal1 = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(mask, kernal1, iterations=4)
    kernal = np.ones((4, 4), np.uint8)
    dilate = cv2.dilate(erosion, kernal, iterations=1)
    img = dilate
    # return(img)
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) != 0:
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        # print(box[0][0],box[0][1],box[3][0],box[3][0])
        return sort(box)
    else:
        return -1


def identify_blue(frame):
    len_list = []
    # 图像预处理
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 转换为HSV格式(色调，饱和度，亮度)
    low_blue = np.array([[95, 43, 46]])
    high_blue = np.array([[130, 255, 255]])  # 设定阈值
    mask = cv2.inRange(hsv, low_blue, high_blue)  # 进行掩模运算
    kernal1 = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(mask, kernal1, iterations=4)
    kernal = np.ones((4, 4), np.uint8)
    dilate = cv2.dilate(erosion, kernal, iterations=1)
    img = dilate
    # return(img)
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) != 0:
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        # print(box[0][0],box[0][1],box[3][0],box[3][0])
        return sort(box)
    else:
        return -1


def identify_green(frame):
    len_list = []
    # 图像预处理
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 转换为HSV格式(色调，饱和度，亮度)
    low_green = np.array([[36, 43, 46]])
    high_green = np.array([[77, 255, 255]])  # 设定阈值
    mask = cv2.inRange(hsv, low_green, high_green)  # 进行掩模运算
    kernal1 = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(mask, kernal1, iterations=4)
    kernal = np.ones((4, 4), np.uint8)
    dilate = cv2.dilate(erosion, kernal, iterations=1)
    img = dilate
    # return(img)
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) != 0:
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        # print(box[0][0],box[0][1],box[3][0],box[3][0])
        return sort(box)
    else:
        return -1


def draw(frame, list_coordinate):
    # return(list_coordinate)
    if list_coordinate[0] != 0:
        x1 = int(list_coordinate[0][0])
        y1 = int(list_coordinate[0][1])
        x2 = int(list_coordinate[0][2])
        y2 = int(list_coordinate[0][3])
        color = str(list_coordinate[1])
    else:
        return frame
    if list_coordinate[0] == 0:
        return frame
    else:
        img = cv2.rectangle(frame, (x2, y2), (x1, y1), (255, 255, 255), 2)
        cv2.putText(
            img, color, (x1, y1 + 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (100, 200, 200), 5
        )
        return img


# while True:
# img = draw(read(),identify_orange(read()))
# img = draw(img,identify_red(read()))
# img = draw(img,identify_yellow(read()))
# img = draw(img,identify_blue(read()))
# img = draw(img,identify_green(read()))
# show(img)
# show(identify(read()))
# if cv2.waitKey(1) & 0xFF == ord('q'):
# break
# print(draw(read(),identify_orange(read())))
    
pub = rospy.Publisher("chatter", Int16MultiArray, queue_size=10)

def talker():
    rospy.init_node("color", anonymous=True)
    pub = rospy.Publisher("chatter", Int16MultiArray, queue_size=10)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)       # topic path needs to be updated
    rospy.spin()

    # rate = rospy.Rate(10)  # 10hz

    # rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
