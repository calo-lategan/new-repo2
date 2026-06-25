#!/usr/bin/env python3
# encoding: utf-8
# Line Following 巡线
import os
import cv2
import math
import time
import rclpy
import queue
import threading
import numpy as np
import sdk.pid as pid
import sdk.misc as misc
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from app.common import ColorPicker
from chassis_msgs.msg import Mecanum
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Image, LaserScan
from interfaces.srv import SetPoint, SetFloat64
from servo_controller_msgs.msg import ServosPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position

class LineFollower:
    def __init__(self, color, node):
        self.node = node
        self.target_lab, self.target_rgb = color
        if os.environ['CAMERA_TYPE'] == 'GEMINI':
            self.rois = ((0.9, 0.95, 0, 1, 0.7), (0.8, 0.85, 0, 1, 0.2), (0.7, 0.75, 0, 1, 0.1))
        else:
            self.rois = ((0.81, 0.83, 0, 1, 0.7), (0.69, 0.71, 0, 1, 0.2), (0.57, 0.59, 0, 1, 0.1))
        self.weight_sum = 1.0

    def get_area_max_contour(self, contours, threshold=100):
        '''
        Get the contour of the largest area.获取最大面积对应的轮廓
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        status = True
        centroid_sum = 0
        h, w = image.shape[:2]
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        for roi in self.rois:
            blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # Intercept roi 截取roi
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # Convert rgb into lab. rgb转lab
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # Perform Gaussian filtering to reduce noise. 高斯模糊去噪
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # Image binarization 二值化
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # Corrode 腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # Dilate 膨胀
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # Find the contour 找轮廓
            max_contour_area = self.get_area_max_contour(contours, 100)  # Get the contour corresponding to the largest contour. 获取最大面积对应轮廓
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # Minimum circumscribed rectangle 最小外接矩形
                box = np.intp(cv2.boxPoints(rect))  # Four corners 四个角
                for j in range(4):
                    box[j, 1] = box[j, 1] + int(roi[0]*h)
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # Draw the rectangle composed of four points. 画出四个点组成的矩形

                # Acquire the diagonal points of the rectangle. 获取矩形对角点
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # Center point of the line. 线的中心点
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # Draw the center point. 画出中心点
                centroid_sum += line_center_x * roi[-1]
            else:
                status = False
            
        if centroid_sum == 0:
            return result_image, None, status
        center_pos = centroid_sum / self.weight_sum  # Calculate the center point according to the ratio. 按比重计算中心点
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # Calculate the line angle. 计算线角度
        return result_image, deflection_angle, status

class LineFollowingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.name = name
        self.set_callback = False
        self.is_running = False
        self.color_picker = None
        self.follower = None
        self.scan_angle = math.radians(45)
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.empty = 0
        self.threshold = 0.5
        self.stop_threshold = 0.4
        self.lock = threading.RLock()
        self.image_sub = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        self.chassis_pub = self.create_publisher(Mecanum, '/chassis_controller/command', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # Publish the image processing result. 图像处理结果发布
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # Subscribe to the camera. 摄像头订阅
        
        self.timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish', callback_group=self.timer_cb_group)
        self.client.wait_for_service()

        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # Start the feature. 开启玩法
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        set_servo_position(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 665), (1, 500)))
        time.sleep(1.0)
        threading.Thread(target=self.main, daemon=True).start()

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("result", result)

            if not self.set_callback:
                self.set_callback = True
                # Set callback function for mouse clicking event. 设置鼠标点击事件的回调函数
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
        self.chassis_pub.publish(Mecanum())
        rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())


    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_target_color")
        with self.lock:
            x, y = request.data.x, request.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(request.data, 5)
                self.chassis_pub.publish(Mecanum())
        response.success = True
        response.message = "set_target_color"
        return response

    def set_running_srv_callback(self, request, response):
        with self.lock:
            self.is_running = request.data
            self.empty = 0

            if self.is_running:
                self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
                self.chassis_pub.publish(Mecanum())
        response.success = True
        response.message = "set_running"
        return response


    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]
        result_image = np.copy(rgb_image)  # The image used to display the result. 显示结果用的画面
        with self.lock:
            # Color picker and line recognition are exclusive. If there is color picker, start picking. 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取
            if self.color_picker is not None:  # Color picker exists. 拾取器存在
                try:
                    target_color, result_image = self.color_picker(rgb_image, result_image)
                    if target_color is not None:
                        self.color_picker = None
                        self.follower = LineFollower(target_color, self)
                        self.get_logger().info("target color: {}".format(target_color))
                except Exception as e:
                    self.get_logger().error(str(e))
            else:
                mecanum = Mecanum()
                mecanum.velocity = 80.0 
                mecanum.direction = 90.0 
                if self.follower is not None:
                    try:
                        result_image, deflection_angle , status= self.follower(rgb_image, result_image, self.threshold)
                        if deflection_angle is not None and self.is_running and  status:
                            self.pid.update(deflection_angle)
                            angular_rate = common.set_range(-self.pid.output, -1, 1)
                            mecanum.angular_rate = misc.map(angular_rate, -0.1, 0.1, -350, 350)
                            self.chassis_pub.publish(mecanum)
                        elif not status:
                            self.chassis_pub.publish(Mecanum())
                        else:
                            self.pid.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))
        if self.image_queue.full():
            # Discard the oldest image if the queue is full. 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # Put image into the queue. 将图像放入队列
        self.image_queue.put(result_image)

def main():
    node = LineFollowingNode('line_following')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

