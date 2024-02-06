# fishmonger 程序技术文档 2024.2.2
## 程序概述
fishmonger 实现了在距离鱼贩子窗口 15cm (±5cm) 时的二维精准定位

## 接口使用
node: `fishmonger`

topic: `fishmonger_cx` 输出`std_msg.msg::Int16`整型表示投放窗口中点的横向x轴坐标

当横坐标约为420时，窗口对准机器人底盘中心
## 实现方法
### 视频帧接入
用`sensor_msgs.msg::Image` subscribe 到`/usb_cam/image_raw` topic的图像输出(`src/usb_cam/launch/usb_cam.launch`文件配置)

逐帧接出视频用`cv_bridge::CvBridge`处理

`frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")`

### OpenCV窗口定位
`process_frame(frame)`

输入: 视频帧

hsv阈值划分 -> 降噪 -> 描边 -> 宽高比筛选

`draw_contour_with_center(frame, contour)`

输入：视频帧，边框

输出：`cx` x纵轴中心点

画最小长方形与计算中心点
