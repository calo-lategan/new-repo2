#!/usr/bin/env python3
# encoding: utf-8

import re
import os
import sys
import time
from jetarm_type_config_ui import Ui_MainWindow
from PyQt5.QtGui import *
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import *
import subprocess

class MainWindow(Ui_MainWindow, QMainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton_save.clicked.connect(self.on_save_clicked)
        self.pushButton_restart_service.clicked.connect(self.on_restart_clicked)
        self.comboBox_robot_type.currentIndexChanged.connect(self.on_robot_type_index_changed)
        self.comboBox_camera_type.currentIndexChanged.connect(self.on_camera_type_index_changed)
        self.comboBox_chassis_type.currentIndexChanged.connect(self.on_chassis_type_index_changed)
        self.comboBox_microphone_type.currentIndexChanged.connect(self.on_microphone_type_index_changed)

    def on_robot_type_index_changed(self, current_index):
        robot_type = self.comboBox_robot_type.currentText()
        if robot_type == 'JetArm Starter':
            self.comboBox_camera_type.setCurrentIndex(0)
        elif robot_type == 'JetArm Advanced':
            self.comboBox_camera_type.setCurrentIndex(1)
        else:
            pass

    def on_camera_type_index_changed(self, current_index):
        pass

    def on_chassis_type_index_changed(self, current_index):
        chassis_type = self.comboBox_chassis_type.currentText()

    def on_microphone_type_index_changed(self, current_index):
        microphone_type = self.comboBox_microphone_type.currentText()

    def on_save_clicked(self):
        camera_type_index = self.comboBox_camera_type.currentIndex()
        camera_type = ["USB_CAM", "GEMINI"][camera_type_index]
        robot_type = self.comboBox_robot_type.currentText().replace(' ', '_').upper()
        asr_language = self.comboBox_asr_language.currentText()
        chassis_type = self.comboBox_chassis_type.currentText()
        microphone_type = self.comboBox_microphone_type.currentText()

        if asr_language == '中文':
            asr_language = 'Chinese'

        data = ""
        with open('/home/ubuntu/ros2_ws/.hiwonderrc', 'r') as file:
            data = file.read()
        data = re.sub(r'export\s*CAMERA_TYPE=.*', 'export CAMERA_TYPE=' + camera_type, data)
        data = re.sub(r'export\s*MACHINE_TYPE=.*', 'export MACHINE_TYPE=' + robot_type, data)
        data = re.sub(r'export\s*ASR_LANGUAGE=.*', 'export ASR_LANGUAGE=' + asr_language, data)
        data = re.sub(r'export\s*CHASSIS_TYPE=.*', 'export CHASSIS_TYPE=' + chassis_type, data)
        data = re.sub(r'export\s*MIC_TYPE=.*', 'export MIC_TYPE=' + microphone_type, data)
        with open('/home/ubuntu/ros2_ws/.hiwonderrc', 'w') as file:
            file.write(data)
        data = ""
        with open('/home/ubuntu/docker/tmp/.hiwonderrc', 'r') as file:
            data = file.read()
        data = re.sub(r'export\s*CAMERA_TYPE=.*', 'export CAMERA_TYPE=' + camera_type, data)
        data = re.sub(r'export\s*MACHINE_TYPE=.*', 'export MACHINE_TYPE=' + robot_type, data)
        data = re.sub(r'export\s*ASR_LANGUAGE=.*', 'export ASR_LANGUAGE=' + asr_language, data)
        data = re.sub(r'export\s*MIC_TYPE=.*', 'export MIC_TYPE=' + microphone_type, data)
        with open('/home/ubuntu/docker/tmp/.hiwonderrc', 'w') as file:
            file.write(data)


    def on_restart_clicked(self):
        self.pushButton_restart_service.setEnabled(False)
        self.pushButton_restart_service.setText("重启中...")
        def callback():
            os.system("systemctl daemon-reload && systemctl restart start_app_node.service && systemctl restart find_device.service")
            self.pushButton_restart_service.setText("重启玩法服务")
            self.pushButton_restart_service.setEnabled(True)
        QTimer.singleShot(1, callback)


if __name__ == "__main__":  
    if os.geteuid() != 0:
        os.system("pkexec python3 " +  os.path.join(os.getcwd(), __file__) + " " + os.environ['DISPLAY'] + " " + os.environ['XAUTHORITY'])
    else:
        if len(sys.argv) == 3:
            os.environ['DISPLAY'] = sys.argv[1]
            os.environ['XAUTHORITY'] = sys.argv[2]
        app = QApplication(sys.argv)
        myshow = MainWindow()
        myshow.show()
        sys.exit(app.exec_())
