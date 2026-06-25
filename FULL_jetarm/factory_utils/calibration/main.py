#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import time
import yaml
import math
import rclpy
import threading
import numpy as np
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from rclpy.node import Node
from ui import Ui_MainWindow
from std_srvs.srv import SetBool, Trigger
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.inverse_kinematics import get_ik, get_position_ik

from kinematics_msgs.srv import SetRobotPose, SetJointValue
from kinematics.kinematics_control import set_pose_target, set_joint_value_target
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition, ServoPosition

tag1_position = [0.185, 0, 0.015]
tag1_space = [0.05, 0.07]
tag2_space = [0.05, 0.07]

tag2_position = [-0.006, 0.16]
space2 = 0.07
height = 0.015

space3 = 0.025
space4 = 0.03

space5 = 0.062
chassis_type = os.environ['CHASSIS_TYPE']
if chassis_type == 'Slide_Rails':
    POSITIONS_YAML_PATH = "/home/ubuntu/ros2_ws/src/stepper/config/calibration.yaml"
else:
    POSITIONS_YAML_PATH = "/home/ubuntu/ros2_ws/src/app/config/calibration.yaml"

init_finish = False

class KinematicsNode(Node):  
    def __init__(self, name):
        global init_finish
        rclpy.init()
        super().__init__(name)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.ik_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        self.ik_client.wait_for_service()
        self.fk_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target')
        self.fk_client.wait_for_service()
        self.start_calibration_client = self.create_client(SetBool, 'calibration/start_calibration')
        self.start_calibration_client.wait_for_service()
        self.enter_client = self.create_client(Trigger, 'calibration/enter')
        self.enter_client.wait_for_service()
        self.start_client = self.create_client(Trigger, 'calibration/start')
        self.start_client.wait_for_service()
        self.exit_client = self.create_client(Trigger, 'calibration/exit')
        self.exit_client.wait_for_service()
        init_finish = True

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enable_calibration(self, status):
        msg = SetBool.Request()
        msg.data = status
        self.send_request(self.start_calibration_client, msg)

    def start_calibration(self):
        msg = Trigger.Request()
        self.send_request(self.start_client, msg)

    def enter_calibration(self):
        msg = Trigger.Request()
        self.send_request(self.enter_client, msg)

    def exit_calibration(self):
        msg = Trigger.Request()
        self.send_request(self.exit_client, msg)

    def bus_servo_set_position(self, t, pulse):
        set_servo_position(self.joints_pub, t, pulse)

    def set_position(self, position, yaw):
        msg = set_pose_target(position, 80, [-90.0, 90.0], 1.0)
        res = self.send_request(self.ik_client, msg)
        if res.pulse:
            servo_pulse = res.pulse

            set_servo_position(self.joints_pub, 0.8, ((1, servo_pulse[0]), ))
            time.sleep(0.8)
            set_servo_position(self.joints_pub, 1.0, ((2, servo_pulse[1]), (3, servo_pulse[2]), (4, servo_pulse[3]), (5, yaw), (10, 200)))
            time.sleep(1.0)
            set_servo_position(self.joints_pub, 0.3, ((10, 500), ))
            time.sleep(0.3)

    def shutdown(self):
        rclpy.shutdown()

class MainWindow(Ui_MainWindow, QMainWindow):
    def __init__(self):
        super().__init__()
        self.current_func_name = ""
        
        self.language = os.environ['ASR_LANGUAGE']
        self.kinematics = None
        threading.Thread(target=self.ros_node, daemon=True).start()
        while not init_finish:
            time.sleep(0.1)
        self.setupUi(self)

        self.pushButton_init.pressed.connect(lambda: self.button_clicked('init'))
        self.pushButton_reset.pressed.connect(lambda: self.button_clicked('reset'))
        self.pushButton_save.pressed.connect(lambda: self.button_clicked('save'))

        self.comboBox.currentIndexChanged.connect(self.index_changed)
        
        with open(POSITIONS_YAML_PATH, 'r') as f:
            self.params = yaml.safe_load(f)
        # print(self.params)
        self.offset_x.valueChanged.connect(lambda value: self.value_changed('offset', 0, value))
        self.offset_y.valueChanged.connect(lambda value: self.value_changed('offset', 1, value))
        self.offset_z.valueChanged.connect(lambda value: self.value_changed('offset', 2, value))
        self.scale_x.valueChanged.connect(lambda value: self.value_changed('scale', 0, value))
        self.scale_y.valueChanged.connect(lambda value: self.value_changed('scale', 1, value))
        self.scale_z.valueChanged.connect(lambda value: self.value_changed('scale', 2, value))
        
        self.button_object = {}
        for i in range(16):
            self.button_object[i] = self.findChild(QPushButton, "pushButton_%d" % (i + 1))
            # 使用默认参数来避免 lambda 捕获引用
            self.button_object[i].pressed.connect(lambda i=i: self.button_clicked(self.button_object[i].text()))

        self.index_changed(0)

    def ros_node(self):
        self.kinematics = KinematicsNode('kinematics') 
        rclpy.spin(self.kinematics)
        self.kinematics.destroy_node()

    def index_changed(self, i):
        # print("选择了第{}项: {}".format(i, self.comboBox.currentText()))
        if i == 0:
            self.kinematics.enter_calibration()
            self.scale_x.setEnabled(False)
            self.scale_y.setEnabled(False)
            self.scale_z.setEnabled(False)
            self.offset_x.setEnabled(False)
            self.offset_y.setEnabled(False)
            self.offset_z.setEnabled(False)
            self.pushButton_reset.setEnabled(False)
            self.pushButton_save.setEnabled(False)
            if self.language == "Chinese":
                self.pushButton_init.setText("标定") 
            if self.language == "English":
                self.pushButton_init.setText("Calibration")
            for i in range(16):
                self.button_object[i].setEnabled(False)
        else:
            self.kinematics.exit_calibration()
            self.kinematics.enable_calibration(False)
            self.scale_x.setEnabled(False)
            self.scale_y.setEnabled(True)
            self.scale_z.setEnabled(False)
            self.offset_x.setEnabled(True)
            self.offset_y.setEnabled(True)
            self.offset_z.setEnabled(True)
            self.pushButton_reset.setEnabled(True)
            self.pushButton_save.setEnabled(True)
            if self.language == "Chinese":
                self.pushButton_init.setText("复位")
            if self.language == "English":
                self.pushButton_init.setText("Init")
            if i == 1:
                for i in range(16):
                    self.button_object[i].setEnabled(True)
                params = self.params.get('kinematics', {})
                # print(params)
                offset = params.get('offset', [0, 0, 0])
                scale = params.get('scale', [1, 1, 1])
            elif i == 2:
                params = self.params.get('pixel', {})
                # print(params)
                offset = params.get('offset', [0, 0, 0])
                scale = params.get('scale', [1, 1, 1])
                for i in range(16):
                    self.button_object[i].setEnabled(False)
            elif i == 3:
                params = self.params.get('depth', {})
                # print(params)
                offset = params.get('offset', [0, 0, 0])
                scale = params.get('scale', [1, 1, 1])
                for i in range(16):
                    self.button_object[i].setEnabled(False)
            self.offset_x.setValue(offset[0])
            self.offset_y.setValue(offset[1])
            self.offset_z.setValue(offset[2])
            self.scale_x.setValue(scale[0])
            self.scale_y.setValue(scale[1])
            self.scale_z.setValue(scale[2])

    def init_pose(self):
        self.kinematics.bus_servo_set_position(0.5, ((10, 200), ))
        time.sleep(0.5)
        self.kinematics.bus_servo_set_position(1.5, ((2, 520), (3, 210), (4, 50), (5, 500)))
        time.sleep(1.0)
        self.kinematics.bus_servo_set_position(0.8, ((1, 500), ))
        time.sleep(0.8)

    def calibration_position(self, position):
        yaw = math.degrees(math.atan2(position[1], position[0]))

        # print(1, yaw)
        if yaw > 45:
            print(1, position)
            yaw = math.degrees(math.atan2(-position[0], position[1]))
            # print(yaw)
            position = [position[0] * self.scale_y.value(), position[1] * self.scale_x.value(), position[2] * self.scale_z.value()]
            position = [position[0] - self.offset_y.value(), position[1] + self.offset_x.value(), position[2] + self.offset_z.value()]
        elif yaw < -45:
            print(2, position)
            yaw = math.degrees(math.atan2(position[0], -position[1]))
            # print(2, yaw)
            position = [position[0] * self.scale_y.value(), position[1] * self.scale_x.value(), position[2] * self.scale_z.value()]
            position = [position[0] + self.offset_y.value(), position[1] - self.offset_x.value(), position[2] + self.offset_z.value()]
        else:
            print(3, position)
            # position[0] -= tag1_position[0]
            position = [position[0] * self.scale_x.value(), position[1] * self.scale_y.value(), position[2] * self.scale_z.value()]
            position = [position[0] + self.offset_x.value(), position[1] + self.offset_y.value(), position[2] + self.offset_z.value()]
        # position[0] += tag1_position[0]
        print(position)
        # if yaw == 90 or yaw == -90:
            # yaw = 0
        yaw = 500 + int(yaw/240*1000)
        return position, yaw

    def button_clicked(self, name):
        position = []
        if name == "init":
            if self.comboBox.currentIndex() == 0:
                self.kinematics.start_calibration()
                self.kinematics.enable_calibration(True)
            else:
                self.init_pose()
        elif name == "reset":
            self.offset_x.setValue(0)
            self.offset_y.setValue(0)
            self.offset_z.setValue(0)
            self.scale_x.setValue(1.0)
            self.scale_y.setValue(1.0)
            self.scale_z.setValue(1.0)
        elif name == "save":
            f = open(POSITIONS_YAML_PATH, 'w', encoding='utf-8')
            yaml.dump(self.params, f)
            self.pushButton_save.setEnabled(False)
        elif name == 'Center':
            self.init_pose()
            position = [tag1_position[0], tag1_position[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'Left Top':
            self.init_pose()
            position = [tag1_position[0] + tag2_space[0], tag1_position[1] + tag2_space[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'Right Top':
            self.init_pose()
            position = [tag1_position[0] + tag2_space[0], tag1_position[1] - tag2_space[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'Left Bottom':
            self.init_pose()
            position = [tag1_position[0] - tag1_space[0], tag1_position[1] + tag1_space[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'Right Bottom':
            self.init_pose()
            position = [tag1_position[0] - tag1_space[0], tag1_position[1] - tag1_space[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'R':
            self.init_pose()
            position = [tag2_position[0] + space2, tag2_position[1] + space2, height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'G':
            self.init_pose()
            position = [tag2_position[0], tag2_position[1] + space2, height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == 'B':
            self.init_pose()
            position = [tag2_position[0] - space2, tag2_position[1] + space2, height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == '1':
            self.init_pose()
            position = [tag2_position[0] - space2, tag2_position[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == '2':
            self.init_pose()
            position = [tag2_position[0], tag2_position[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == '3':
            self.init_pose()
            position = [tag2_position[0] + space2, tag2_position[1], height]
            position, yaw = self.calibration_position(position)
            self.kinematics.set_position(position, yaw)
        elif name == '':
            self.init_pose()
            position = [tag2_position[0], -tag2_position[1], height]
            position, yaw = self.calibration_position(position)
            # print(position, yaw)
            self.kinematics.set_position(position, yaw)
        elif name == 'Residual Waste':
            self.init_pose()
            position = [space5 + space3, -tag2_position[1] - space5, height]
            position, yaw = self.calibration_position(position)
            # print(position, yaw)
            self.kinematics.set_position(position, yaw)
        elif name == 'Food Waste':
            self.init_pose()
            position = [space3, -tag2_position[1] - space5, height]
            position, yaw = self.calibration_position(position)
            # print(position, yaw)
            self.kinematics.set_position(position, yaw)
        elif name == 'Hazardous Waste':
            self.init_pose()
            position = [tag2_position[0] - space4, -tag2_position[1] - space5, height]
            position, yaw = self.calibration_position(position)
            # print(position, yaw)
            self.kinematics.set_position(position, yaw)
        elif name == 'Recyclable Waste':
            self.init_pose()
            position = [tag2_position[0] - space4 - space5, -tag2_position[1] - space5, height]
            position, yaw = self.calibration_position(position)
            # print(position, yaw)
            self.kinematics.set_position(position, yaw) 
        # print(position)

    def value_changed(self, key, idx, value):
        if self.comboBox.currentIndex() == 1:
            self.params['kinematics'][key][idx] = value
        elif self.comboBox.currentIndex() == 2:
            self.params['pixel'][key][idx] = value
        elif self.comboBox.currentIndex() == 3:
            self.params['depth'][key][idx] = value
        if self.pushButton_save.isEnabled() == False:
            self.pushButton_save.setEnabled(True)

if __name__ == "__main__":  
    app = QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())

