#!/usr/bin/python3
# coding=utf8
import os
import cv2
import time
import numpy as np
import sdk.pid as pid

class ObjectTracker:
    def __init__(self, use_mouse=False, automatic=False, log=None): 
        self.log = log
        self.miss_count = 0
        self.detect_count = 0
        self.detect = True
        self.start_track = False
        self.automatic = automatic
        self.use_mouse = use_mouse
        if self.use_mouse:
            name = 'image'
            # cv2.namedWindow(name, 1)
            cv2.setMouseCallback(name, self.onmouse)

        self.mouse_click = False
        self.selection = None  # Real-time tracking region of the mouse. 实时跟踪鼠标的跟踪区域
        self.track_window = None  # Region where the object to be detected is located. 要检测的物体所在区域
        self.drag_start = None  # Flag indicating whether mouse dragging has started. 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False
        self.stop_track = False
        
        self.joint4_dis = 233

        self.y_dis = 500
        self.z_dis = 0.15
        
        self.joint_pid = pid.PID(0.0, 0.0, 0.0)
        self.z_pid = pid.PID(0.0, 0.0, 0.0)# pid initialization pid初始化
        self.y_pid = pid.PID(0.0, 0.0, 0.0)

    def set_init_param(self, joint4_dis, y_dis, z_dis): 
        self.joint4_dis = joint4_dis
        self.y_dis = y_dis
        self.z_dis = z_dis

    def update_pid(self, p1, p2, p3):
        self.joint_pid = pid.PID(p1[0], p1[1], p1[2])
        self.z_pid = pid.PID(p2[0], p2[1], p2[2])# pid initialization pid初始化
        self.y_pid = pid.PID(p3[0], p3[1], p3[2])

    # Mouse click event callback function. 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button pressed. 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # Starting position of the mouse. 鼠标起始位置
            self.track_window = None
        if self.drag_start:  # Start dragging the mouse and record its position. 是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # Left mouse button released. 鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None  # Real-time tracking region of the mouse. 实时跟踪鼠标的跟踪区域
            self.track_window = None  # Region where the object to be detected is located. 要检测的物体所在区域
            self.drag_start = None  # Flag indicating whether mouse dragging has started. 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False

    def set_track_target(self, tracker, target, image):
        self.stop_track = False
        self.start_circle = False
        self.start_track = True
        tracker.init(image, target)

    def stop(self):
        self.stop_track = True
        self.start_circle = False

    def get_target(self, tracker, image):
        if self.start_circle and self.use_mouse and not self.automatic:
            # Specify a region by dragging a box with the mouse. 用鼠标拖拽一个框来指定区域
            h, w = image.shape[:2]
            if self.track_window:  # After drawing the tracking window, mark the target in real time. 跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:  # Tracking window updates in real time as the mouse is dragged. 跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 255, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click:
                if not self.mouse_click:
                    self.start_circle = False
            if not self.start_circle:
                self.log.info('start tracking')
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                # print(bbox)
                tracker.init(image, bbox)
                self.start_track = True
        else:
            if not self.start_circle:
                if not self.stop_track:
                    ok, box = tracker.track(image)
                    # print(ok, box)
                    # self.log.info(f'{ok} {box}')
                    if self.miss_count > 3:
                        self.miss_count = 0
                        self.detect = False
                    if self.detect_count > 3:
                        self.detect_count = 0
                        self.detect = True
                    if ok > 0.7:
                        self.detect_count += 1
                        self.miss_count = 0
                        if self.detect:
                            return image, box
                        else:
                            cv2.putText(image, "Tracking failure detected !", (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 255), 1)
                    else:
                        self.detect_count = 0
                        self.miss_count += 1
                        # pass
                        # Tracking failure
                        cv2.putText(image, "Tracking failure detected !", (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 255), 1)
        return image, None

    def track(self, tracker, image):
        image, box = self.get_target(tracker, image)
        # print(box)
        if box is not None:
            img_h, img_w = image.shape[:2]
            p1 = (int(box[0]), int(box[1]))
            p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))

            cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
            center_x = (p1[0] + p2[0]) / 2
            center_y = (p1[1] + p2[1]) / 2

            x, y = center_x, center_y
            
            if self.z_dis <= 0.23:
                self.joint_pid.SetPoint = img_h / 2.0
                if abs(y - img_h / 2.0) < 25:
                    y = img_h / 2.0
                self.joint_pid.update(y)
                self.joint4_dis += self.joint_pid.output
                self.joint4_dis = 100 if self.joint4_dis < 100 else self.joint4_dis
                self.joint4_dis = 391 if self.joint4_dis > 391 else self.joint4_dis
            
            if self.joint4_dis >= 391 or self.z_dis > 0.23:
                self.z_pid.SetPoint = img_h / 2  # Set 设定
                self.z_pid.update(y)  # Current 当前
                self.z_dis += self.z_pid.output  # Output 输出

                self.z_dis = 0.23 if self.z_dis < 0.23 else self.z_dis
                self.z_dis = 0.3 if self.z_dis > 0.3 else self.z_dis
            if abs(x - img_w/2.0) < 15:
                x = img_w / 2.0
            self.y_pid.SetPoint = img_w / 2.0
            self.y_pid.update(x)
            self.y_dis += self.y_pid.output

            self.y_dis = 200 if self.y_dis < 200 else self.y_dis
            self.y_dis = 800 if self.y_dis > 800 else self.y_dis
            # print(self.z_dis, y, self.joint4_dis)
        return int(self.y_dis), self.z_dis, int(self.joint4_dis), image

if __name__ == '__main__':
    cap = cv2.VideoCapture(-1)
    track = ObjectTracker(True)
    while True:
        try:
            ret, image = cap.read()
            if ret:
                x, y, frame = track.track(image, None)
                cv2.imshow('image', frame)
                cv2.waitKey(1)
            else:
                time.sleep(0.01)
        except KeyboardInterrupt:
            break
    cap.release()
    cv2.destroyAllWindows()



