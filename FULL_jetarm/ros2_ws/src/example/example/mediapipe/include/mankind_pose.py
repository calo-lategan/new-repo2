#!/usr/bin/env python3
# encoding: utf-8
# Limb Skeleton Detection 肢体骨骼检测
"""
This program implements human skeleton recognition. 这个程序实现了人体骨架识别
Runtime behavior: The desktop displays the recognition result screen, showing the human skeleton lines. 运行现象：桌面显示识别结果画面， 显示人体骨架连线
"""
import cv2
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position


class MankindPoseNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        # Instantiate a limb recognizer. 实例化一个肢体识别器
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.2
        )
        self.drawing = mp.solutions.drawing_utils # Result drawing tool 结果绘制工具
        self.fps = fps.FPS() # frame rate calculator 帧率计数器
        self.bridge = CvBridge()  # Used for the conversion between ROS Image messages and OpenCV images. 用于ROS Image消息与OpenCV图像之间的转换

        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # Servo control 舵机控制
        # Wait for the service to start. 等待服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        set_servo_position(self.joints_pub, 1.5, ((10, 500), (5, 500), (4, 330), (3, 100), (2, 700), (1, 500)))

        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # If the queue is full, discard the oldest image. 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # Put the image into the queue. 将图像放入队列
        self.image_queue.put(bgr_image)

    def main(self):
        while True:
            bgr_image = self.image_queue.get()
            bgr_image = cv2.flip(bgr_image, 1)  # Mirror image, aligned with the screen and camera for better visualization. 镜像画面, 这样可以正对屏幕和相机看效果
            result_image = np.copy(bgr_image) # Duplicate the image as the result canvas, and draw the results on it. 将画面复制一份作为结果，结果绘制在这上面
            # bgr_image = cv2.resize(bgr_image, (int(ros_image.width / 2), int(ros_image.height / 2))) 
            results = self.pose.process(bgr_image)  # Perform recognition 进行识别
            self.drawing.draw_landmarks(result_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS) # Draw the joints and lines connecting them. 画出各关节及连线
            self.fps.update() # Calculate frame rate 计算帧率
            self.fps.show_fps(result_image)
            cv2.imshow("result", result_image)
            cv2.waitKey(1)

        cv2.destroyAllWindows()
        rclpy.shutdown()


def main():
    node = MankindPoseNode('mankind_pose')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
    

