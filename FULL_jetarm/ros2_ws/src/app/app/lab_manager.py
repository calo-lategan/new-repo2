#!/usr/bin/env python3

"""
This program provides services such as threshold adjustment and saving functionality. 此程序提供了阈值调节,保存等功能的服务
"""
import os
import sys
import cv2
import time
import yaml
import rclpy
import queue
import threading
import numpy as np
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter
from std_srvs.srv import Empty, Trigger, SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import StashRange, GetRange, ChangeRange, GetAllColorName


class LabConfigManagerNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Read the required parameters. 读取需要的参数
        self.config_file_path = "/home/ubuntu/ros2_ws/src/app/config/lab_config.yaml"
        self.color_ranges = self.get_parameters_by_prefix('color_range_list')
        self.kernel_erode = 5
        self.kernel_dilate = 5
        self.current_range = {'min': [0, 0, 0], 'max': [100, 100, 100]}
        self.image_queue = queue.Queue(maxsize=2)
        self.bridge = CvBridge()  # Used for the conversion between ROS Image messages and OpenCV images. 用于ROS Image消息与OpenCV图像之间的转换
        if 'red' in self.color_ranges:
            self.current_range = self.color_ranges['red']

        self.thread_started = False 
        # Visual-related topics 画面相关的topic
        self.image_sub = None

        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)


        # Enter, exit, start, stop service. 进入, 退出, 启动, 停止 服务
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.running_srv = self.create_service(SetBool, '~/set_running',  self.set_running_srv_callback)

        # Modify threshold, maintain threshold and other related services 修改阈值, 保持阈值等服务
        self.save_to_disk_srv = self.create_service(Trigger, '~/save_to_disk',  self.save_to_disk_srv_callback)
        self.get_color_range_srv = self.create_service(GetRange, '~/get_range',  self.get_range_srv_callback)
        self.change_range_srv = self.create_service(ChangeRange, '~/change_range',  self.change_range_srv_callback)
        self.stash_range_srv = self.create_service(StashRange, '~/stash_range',  self.stash_range_srv_callback)
        self.get_all_color_name_srv = self.create_service(GetAllColorName, '~/get_all_color_name',
self.get_all_color_name_srv_callback)

        Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # Heartbeat package 心跳包
    def image_callback(self, ros_image):
        """
        Camera screen callback 相机画面回调
        :params ros_image: Frame data 画面数据
        """
        # Convert the ros format image to opencv format. 将ros格式图像转换为opencv格式
        # image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        image = np.array(cv_image, dtype=np.uint8)
        if not self.thread_started:
            # Start the thread only on the first call. 只在第一次调用时启动线程
            threading.Thread(target=self.image_processing, args=(ros_image,), daemon=True).start()
            self.thread_started = True 

        if self.image_queue.full():
            # # Discard the oldest image if the queue is full. 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # # Put image into the queue. 将图像放入队列
        self.image_queue.put(image)
        
    def image_processing(self, ros_image):
        while True:
            range_ = self.current_range
            image = self.image_queue.get()
            ow, oh = image.shape[1], image.shape[0] 
            # Perform image processing tasks such as binarization and other operations. 对图像进行处理做二值化等操作
            image_resize = cv2.resize(image, (int(ow/2), int(oh/2)), interpolation=cv2.INTER_NEAREST)
            frame_result = cv2.cvtColor(image_resize, cv2.COLOR_RGB2LAB)
            frame_result = cv2.GaussianBlur(frame_result, (3, 3), 3)
            mask = cv2.inRange(frame_result, tuple(range_['min']), tuple(range_['max']))  # Perform bitwise operations on the original image and the mask. 对原图像和掩模进行位运算
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_erode, self.kernel_erode)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_dilate, self.kernel_dilate)))

            # Publish the processed binarized image. 将处理后的二值化图像发布
            rgb_image = cv2.resize(dilated, (ow, oh))
            # rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB).tobytes()
            # ros_image.data = rgb_image
            # self.result_publisher.publish(ros_image)

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(rgb_image, "mono8"))


    def enter_srv_callback(self, request, response):
        """
        APP enter function APP 进入功能
        Subscribe to camera feeds 注册对相机的订阅
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'enter')
        self.thread_started = False 
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)

        response.success = True
        response.message = "start"
        return response
    

    def exit_srv_callback(self, request, response):
        """
        APP exit function APP退出功能
        It will unsubscribe from the camera feed and stop the heartbeat timer. 会注销掉相机的订阅, 并停止心跳定时器
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'exit')
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))

        response.success = True
        response.message = "exit"
        return response
    
    
    def set_running_srv_callback(self, request, response):
        """
        Originally intended to control running or pausing operations, but it has been deprecated here. 本来时用来控制运行或者暂停运行的, 但是这里废弃了
        To maintain compatibility, this empty function is retained. 为了保持兼容性留着这个空函数
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set running called')
        response.success = True
        response.message = "start"
        return response
    

    def save_to_disk_srv_callback(self, request, response):
        """
        Save the current list of thresholds to the hard disk (SD card). 保存当前的阈值列表到硬盘(sd卡)中
        """

        try:
            # Read existing configuration files. 读取现有配置文件
            with open(self.config_file_path, 'r') as f:
                existing_config = yaml.safe_load(f)
        
            # Obtain existing color_range_list. 获取现有的 color_range_list
            color_range_dict = existing_config.get('/**', {}).get('ros__parameters', {}).get('color_range_list', {})

            # Renew modified values. 更新修改过的值
            color_range_dict[self.param_name] = self.param_value

            #
            # 9/5000
            # Construct the final parameter format. 构造最终的参数格式
            final_config = {
                '/**': {
                    'ros__parameters': {
                        'color_range_list': color_range_dict
                    }
                }
            }

            # Save in YAML format. 保存为 YAML 格式
            with open(self.config_file_path, 'w') as f:
                yaml.dump(final_config, f, default_flow_style=False, allow_unicode=True)

            self.get_logger().info("Successfully saved color ranges to disk.")
    
        except Exception as e:
            self.get_logger().error(f"Error saving color ranges to disk: {e}")
        

        response.success = True
        response.message = "Thresholds saved successfully."
        return response
    
    def get_range_srv_callback(self, request, response):
        """
        Get specified color threshold. 获取指定颜色的阈值
        """
        try:
            # Read existing configuration files. 读取现有配置文件
            with open(self.config_file_path, 'r') as f:
                config = yaml.safe_load(f)

            # Get the dictionary of color_range_list. 获取 color_range_list 的字典
            color_range_dict = config.get('/**', {}).get('ros__parameters', {}).get('color_range_list', {})

            if request.color_name in color_range_dict:
                color_range = color_range_dict[request.color_name]
                response.min = color_range.get('min', [])
                response.max = color_range.get('max', [])
                response.success = True
            else:
                self.get_logger().warn(f"Color range for '{request.color_name}' not found.")
                response.success = False
                response.min = []
                response.max = []
    
        except Exception as e:
            self.get_logger().error(f"Error reading color range from YAML: {e}")
            response.success = False
            response.min = []
            response.max = []
            self.get_logger().info(str(response)) 
        return response
    
    def change_range_srv_callback(self,  request, response):
        """
        Modify current color threshold. 修改当前的颜色阈值
        Change the control result frame. 就是控制结果画面改变
        :param msg: msg.min lower limit: msg.min, upper limit: msg.max. msg.min 阈值下限， msg.max 阈值上限
        """
        self.current_range = dict(min=list(request.min), max=list(request.max))
        # self.get_logger().info(f"self.current_range '{self.current_range}'.")
        response.success = True
        response.message = "start"
        return response
    
    

    def stash_range_srv_callback(self, request, response):
        """
        Temporarily store the current threshold.暂存当前阈值
        Modify the threshold of the specified color to the current threshold (i.e., the threshold corresponding to the resulting image). 修改指定颜色的阈值为当前阈值(就是结果画面对应的阈值)
        :param msg: msg.color_name, color name to be modified.要修改的颜色名称
        """
        color_ranges = self.get_parameters_by_prefix('color_range_list')
        color_ranges[request.color_name] = self.current_range  # Modify specified color threshold. 修改指定颜色的阈值
        # self.get_logger().info(f"self.current_range '{self.current_range}'.")
        params = []
        for param_name, param_value in color_ranges.items():
        
            # Create a new parameter object for each Parameter. 对每个参数，创建一个新的 Parameter 对象
            if isinstance(param_value, (int, float, str)):  # Make sure that the parameter values are of the supported types. 确保参数值是支持的类型
                new_param = Parameter(param_name, Parameter.Type.STRING, str(param_value))  # Change here to ensure that the parameter types are correct. 更改这里以确保参数类型正确
                params.append(new_param)
            elif isinstance(param_value, list):  # If it is a list type. 如果是列表类型
                # Suppose the color range is a dictionary containing 'min' and 'max', and each value in the list should be an integer. 假设颜色范围是一个包含 `min` 和 `max` 的字典，并且列表中的每个值都应为整数
                min_range = param_value[0]  # Suppose each value is a list and take its' min 'and' max '. 假设每个值是列表，取其 `min` 和 `max`
                max_range = param_value[1]
                new_param_min = Parameter(param_name + '.min', Parameter.Type.INTEGER_ARRAY, min_range)
                new_param_max = Parameter(param_name + '.max', Parameter.Type.INTEGER_ARRAY, max_range)
                params.append(new_param_min)
                params.append(new_param_max)
            else:
                self.get_logger().warn(f"Unsupported parameter type for {param_name}: {type(param_value)}")
        # Set parameters. 设置参数
        self.param_name = param_name
        self.param_value = param_value

        response.success = True
        response.message = "start"
        return response
    
    def get_all_color_name_srv_callback(self, request, response):
        """
        Retrieve the names of all saved colors. 获取保存的全部颜色的名称
        """
        ranges  = self.get_parameters_by_prefix('color_range_list')# Get all color threshold from parameter server.  从参数服务器获取所有颜色阈值
        color_names = [key.split('.')[0] for key in ranges.keys()]  # Extract the color name from the parameter name. 从参数名中提取颜色名称
        # self.get_logger().info(f"color_names '{color_names}'.") 
        response.color_names = color_names
        return response
    



def main():
    node = LabConfigManagerNode('lab_config_manager')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
