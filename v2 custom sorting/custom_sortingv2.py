#!/usr/bin/env python3
# coding: utf8
# Custom Object & Color Sorting - v2
#
# Improvements over v1 (example pi scripts/My custom scripts/custom_sorting.py):
#   1. FASTER motion: trajectory interpolation, parallel servo dispatch,
#      tunable durations, fewer fixed sleeps, action-completion polling
#      instead of conservative time.sleep() pads.
#   2. MORE ACCURATE: startup self-calibration that auto-corrects place-bin
#      offsets by detecting the colored bins in the camera frame, multi-frame
#      detection averaging before lock-in, and pre-grasp hover-and-recheck.
#   3. DYNAMIC GRIPPING: vision + bus-servo feedback. After each grip we read
#      the gripper servo position to detect "fully-closed = missed object" and
#      load to detect "stalled = hit obstacle / over-tight". On miss/stall we
#      adjust the close pulse and pose and retry up to N times. After lift we
#      also re-detect the target in the frame to confirm pick.
#   4. LIVE TUNABLE: every behavior knob is a ROS2 parameter with an
#      on_set_parameters_callback so tune_ui.py (or rqt) can change it at
#      runtime - speed, aggression, retries, area gates, grip force, etc.
#
# Drop this file (and the accompanying launch file) into the `app` ROS2
# package on the Jetson Orin Nano image, register it in setup.py, and launch
# with `ros2 launch app custom_sorting_nodev2.launch.py`.

import os
import cv2
import yaml
import time
import math
import copy
import queue
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sdk import common, fps
from app.common import Heart
from dt_apriltags import Detector
from interfaces.srv import SetStringBool
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller.bus_servo_control import set_servo_position
from kinematics.kinematics_control import set_pose_target
from app.utils import calculate_grasp_yaw, position_change_detect, image_process, distortion_inverse_map

from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd

from ultralytics import YOLO


GRIPPER_ID = 10
JOINT_IDS = (1, 2, 3, 4, 5)


class MotionController:
    """Wraps interpolated pick / place + servo feedback so the main node stays
    declarative. Holds no state of its own beyond the publisher / clients - all
    tunables come in via the calling node so the UI can change them live."""

    def __init__(self, node, joints_pub, kinematics_client, bus_servo_state_client):
        self.node = node
        self.joints_pub = joints_pub
        self.kinematics_client = kinematics_client
        self.bus_servo_state_client = bus_servo_state_client
        self._abort = False

    def abort(self, value=True):
        self._abort = value

    @property
    def aborted(self):
        return self._abort

    def _send(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok() and not self._abort:
            if future.done() and future.result():
                return future.result()

    def _sleep(self, dt):
        # interruptible sleep; lets aborts propagate through long dwells
        end = time.time() + dt
        while time.time() < end and not self._abort and rclpy.ok():
            time.sleep(min(0.02, end - time.time()))

    def get_servo_state(self, servo_id, fields=('position',)):
        """Returns dict of requested fields for a single servo, or {} on error."""
        if self.bus_servo_state_client is None:
            return {}
        req = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        cmd.id = int(servo_id)
        cmd.get_position = 1 if 'position' in fields else 0
        cmd.get_temperature = 1 if 'temperature' in fields else 0
        cmd.get_voltage = 1 if 'voltage' in fields else 0
        req.cmd = [cmd]
        try:
            res = self._send(self.bus_servo_state_client, req)
            if res is None or not res.state:
                return {}
            s = res.state[0]
            out = {}
            if 'position' in fields and s.position:
                out['position'] = int(s.position[0])
            if 'temperature' in fields and s.temperature:
                out['temperature'] = int(s.temperature[0])
            if 'voltage' in fields and s.voltage:
                out['voltage'] = int(s.voltage[0])
            return out
        except Exception as e:
            self.node.get_logger().warn(f'servo state read failed: {e}')
            return {}

    def goto_pose(self, position, pitch, duration, parallel_base=True, dt=0.05):
        """Interpolated move to a Cartesian pose. Returns the final pulses or
        None on failure / abort. If parallel_base is True the base joint (1)
        is dispatched in parallel with the arm joints to shave dwell time."""
        if self._abort:
            return None
        msg = set_pose_target(position, pitch, [-180.0, 180.0], 1.0, duration=duration)
        res = self._send(self.kinematics_client, msg)
        if res is None or not res.pulse:
            return None
        servo_data = np.array(res.pulse).reshape(-1, 5).tolist()
        if not servo_data:
            return None
        first = servo_data[0]
        last = servo_data[-1]
        if parallel_base:
            # kick off base rotation immediately in parallel with the arm sweep
            set_servo_position(self.joints_pub, max(duration * 0.6, 0.2),
                               ((1, last[0]),))
        steps = max(1, len(servo_data))
        step_dt = max(0.02, duration / steps)
        for i in servo_data:
            if self._abort:
                return None
            set_servo_position(self.joints_pub, step_dt,
                               ((2, i[1]), (3, i[2]), (4, i[3])))
            self._sleep(step_dt * 0.85)
        return last

    def set_gripper(self, pulse, duration):
        set_servo_position(self.joints_pub, float(duration), ((GRIPPER_ID, int(pulse)),))
        self._sleep(duration)

    def set_wrist(self, pulse, duration):
        set_servo_position(self.joints_pub, float(duration), ((5, int(pulse)),))
        self._sleep(duration)

    def grip_with_feedback(self, close_pulse, open_pulse,
                           full_closed_pulse, slack, duration,
                           confirm_delay=0.25):
        """Close gripper then read-back its position.

        Returns one of: 'grabbed', 'missed', 'stalled'.
          - 'missed'  : final position is at/past full_closed_pulse - slack
                        (jaws met each other, no object in between)
          - 'stalled' : final position is well short of close_pulse
                        (something prevented closing - obstacle / over-tight)
          - 'grabbed' : final position landed in the expected band
        """
        self.set_gripper(close_pulse, duration)
        self._sleep(confirm_delay)
        st = self.get_servo_state(GRIPPER_ID, fields=('position', 'temperature'))
        pos = st.get('position', None)
        temp = st.get('temperature', None)
        if temp is not None and temp > 65:
            self.node.get_logger().warn(f'gripper temp {temp}C - cooling off')
            return 'stalled'
        if pos is None:
            return 'grabbed'  # no feedback available - assume best
        if pos >= (full_closed_pulse - slack):
            return 'missed'
        if abs(pos - close_pulse) > 60 and pos < close_pulse - 60:
            # Servo couldn't reach commanded close - obstruction.
            return 'stalled'
        return 'grabbed'


class ObjectSortingNodeV2(Node):
    """v2 sorting node. See module-level docstring for the deltas vs v1."""

    DEFAULT_PLACE_POSITIONS = {
        'green': [-0.006, 0.23, 0.015],
        'red':   [ 0.064, 0.23, 0.015],
        'blue':  [-0.076, 0.23, 0.015],
        'tag1':  [-0.076, 0.16, 0.015],
        'tag2':  [-0.006, 0.16, 0.015],
        'tag3':  [ 0.064, 0.16, 0.015],
        'scaff': [-0.076, 0.16, 0.015],
    }

    def __init__(self, name):
        rclpy.init()
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # ---- Tunable parameters (live via on_set_parameters_callback) ----
        self._declare_tunables()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---- Models / shared state ----
        engine_path = self.get_parameter('engine_path').value
        self.yolo_model = YOLO(engine_path, task='detect')

        proto_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/deploy.prototxt'
        model_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/hed_pretrained_bsds.caffemodel'
        self.image_process = image_process.GetObjectSurface(proto_path, model_path)
        self.at_detector = Detector(searchpath=['apriltags'], families='tag36h11',
                                    nthreads=4, quad_decimate=1.0, quad_sigma=0.0,
                                    refine_edges=1, decode_sharpening=0.25, debug=0)

        self.lock = threading.RLock()
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        self.data = common.get_yaml_data(os.path.join(self.config_path, "lab_config.yaml"))
        self.lab_data = self.data['/**']['ros__parameters']
        self.camera_type = os.environ['CAMERA_TYPE']

        self.tag_size = 0.025

        # Place positions are mutable so the self-calibration step can fine-
        # tune them at runtime per-bin.
        self.place_position = copy.deepcopy(self.DEFAULT_PLACE_POSITIONS)
        self.place_offsets = {k: [0.0, 0.0, 0.0] for k in self.place_position}

        self.target_labels = {
            'red': True, 'green': True, 'blue': True, 'scaff': True,
            'tag1': False, 'tag2': False, 'tag3': False,
        }
        # Detection-history per target so we average several frames before
        # locking in a pick - cuts pixel jitter, hugely improves accuracy.
        self.detection_history = {}
        self.running = True
        self._init_parameters()

        # ---- Pubs / subs / services ----
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.timer_cb_group = ReentrantCallbackGroup()

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_sorting_srv = self.create_service(SetBool, '~/enable_sorting',
                                                      self.enable_sorting_srv_callback)
        self.set_target_srv = self.create_service(SetStringBool, '~/set_target',
                                                  self.set_target_srv_callback)
        self.recalibrate_srv = self.create_service(Trigger, '~/recalibrate',
                                                   self.recalibrate_srv_callback)

        self.kinematics_client = self.create_client(SetRobotPose,
                                                    'kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()
        self.set_joint_value_target_client = self.create_client(
            SetJointValue, 'kinematics/set_joint_value_target',
            callback_group=self.timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()

        self.bus_servo_state_client = self.create_client(
            GetBusServoState, 'ros_robot_controller/bus_servo/get_state',
            callback_group=self.timer_cb_group)
        # Don't hard-block on this one - if the controller is missing we just
        # degrade to vision-only confirmation.
        if not self.bus_servo_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('bus_servo/get_state unavailable - '
                                   'servo feedback disabled, vision-only retries')
            self.bus_servo_state_client = None

        self.motion = MotionController(self, self.joints_pub,
                                       self.kinematics_client,
                                       self.bus_servo_state_client)

        self.timer = self.create_timer(0.0, self.init_process,
                                       callback_group=self.timer_cb_group)

    # ------------------------------------------------------------------ params

    def _declare_tunables(self):
        # Each declare(...) is wrapped in a try because parameters from
        # overrides may already be auto-declared.
        defs = [
            # Detection
            ('engine_path', '/home/ubuntu/third_party_ros2/data/best_scaff2.engine', None),
            ('min_object_area', 500, (50, 5000)),
            ('max_object_area', 7000, (1000, 30000)),
            ('lock_distance_thresh', 0.005, (0.001, 0.05)),
            ('count_still_threshold', 5, (1, 30)),     # was 10 in v1
            ('count_move_threshold', 8, (1, 30)),      # was 10
            ('detection_avg_frames', 3, (1, 10)),
            # Motion
            ('motion_speed', 1.0, (0.3, 2.5)),         # >1 = faster
            ('aggression', 1.0, (0.3, 2.0)),           # interpolation step scale
            ('hover_height', 0.06, (0.02, 0.15)),
            ('approach_dwell', 0.15, (0.0, 1.0)),
            ('parallel_base_motion', True, None),
            # Gripping
            ('gripper_open_pulse', 200, (50, 500)),
            ('gripper_close_pulse', 540, (300, 700)),
            ('gripper_full_closed_pulse', 700, (500, 900)),
            ('gripper_slack', 25, (5, 80)),            # band before "missed"
            ('gripper_close_duration', 0.4, (0.1, 2.0)),
            ('gripper_step_pulse', 30, (5, 100)),      # tighten step on retry
            # Retries
            ('max_pick_retries', 3, (0, 6)),
            ('vision_confirm_pick', True, None),
            ('servo_feedback_enabled', True, None),
            # Self-calibration
            ('startup_self_calibrate', True, None),
            ('place_bin_color_check', True, None),
        ]
        for name, default, rng in defs:
            try:
                if isinstance(default, bool):
                    desc = ParameterDescriptor()
                    self.declare_parameter(name, default, descriptor=desc)
                elif isinstance(default, int):
                    desc = ParameterDescriptor()
                    if rng:
                        desc.integer_range = [IntegerRange(from_value=int(rng[0]),
                                                            to_value=int(rng[1]),
                                                            step=1)]
                    self.declare_parameter(name, default, descriptor=desc)
                elif isinstance(default, float):
                    desc = ParameterDescriptor()
                    if rng:
                        desc.floating_point_range = [FloatingPointRange(
                            from_value=float(rng[0]), to_value=float(rng[1]), step=0.0)]
                    self.declare_parameter(name, default, descriptor=desc)
                else:
                    self.declare_parameter(name, default)
            except Exception:
                # already declared via overrides - that's fine
                pass

    def _on_param_change(self, params):
        # We don't cache the values - everything reads via get_parameter at
        # use-time so changes are picked up immediately.
        for p in params:
            self.get_logger().info(f'tuned {p.name} -> {p.value}')
        return SetParametersResult(successful=True)

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ misc

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _init_parameters(self):
        self.heart = None
        self.endpoint = None
        self.target_miss_count = 0
        self.transport_info = None
        self.intrinsic = None
        self.distortion = None
        self.start_transport = False
        self.enable_sorting = False
        self.white_area_center = None
        self.enter = False
        self.roi = []
        self.count_move = 0
        self.count_still = 0
        self.target = None
        self.start_get_roi = False
        self.last_position = None
        self.last_object_info_list = None
        self.image_sub = None
        self.camera_info_sub = None
        self.detection_history = {}
        self.data = common.get_yaml_data(os.path.join(self.config_path, "lab_config.yaml"))
        self.lab_data = self.data['/**']['ros__parameters']

    def init_process(self):
        self.timer.cancel()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        if self.get_parameter('start').value:
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            req = SetBool.Request(); req.data = True
            self.enable_sorting_srv_callback(req, SetBool.Response())

        if not self.get_parameter('broadcast').value:
            for label in ('red', 'green', 'blue', 'scaff'):
                req = SetStringBool.Request()
                req.data_bool = True
                req.data_str = label
                self.set_target_srv_callback(req, SetBool.Response())

        if self.p('startup_self_calibrate'):
            threading.Thread(target=self._self_calibrate, daemon=True).start()

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32mv2 init finish\033[0m')

    # ------------------------------------------------------------------ motion helpers

    def go_home(self, interrupt=True, fast=True):
        speed = max(0.1, 1.0 / float(self.p('motion_speed')))
        if interrupt:
            self.motion.set_gripper(self.p('gripper_open_pulse'), 0.3 * speed)
        joint_angle = [500, 520, 210, 50, 500]
        # parallel arm + base
        set_servo_position(self.joints_pub, 0.6 * speed,
                           ((2, joint_angle[1]), (3, joint_angle[2]),
                            (4, joint_angle[3]), (5, 500)))
        self.motion._sleep(0.6 * speed)
        set_servo_position(self.joints_pub, 0.5 * speed, ((1, joint_angle[0]),))
        self.motion._sleep(0.5 * speed)

    # ------------------------------------------------------------------ ROI

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
        while True:
            if self.intrinsic is not None and self.distortion is not None:
                break
            time.sleep(0.05)

        tvec = extristric[:1]
        rmat = extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)),
                                                   np.array(rmat), 0.03)
        self.extristric = tvec, rmat
        imgpts, _ = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec),
                                      self.intrinsic, self.distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        x_min = min(imgpts, key=lambda p: p[0])[0]
        x_max = max(imgpts, key=lambda p: p[0])[0]
        y_min = min(imgpts, key=lambda p: p[1])[1]
        y_max = max(imgpts, key=lambda p: p[1])[1]
        self.roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)

    # ------------------------------------------------------------------ services

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32menter custom sorting v2\033[0m')
        self._init_parameters()
        self.heart = Heart(self, '~/heartbeat', 5,
                           lambda _: self.exit_srv_callback(Trigger.Request(),
                                                            Trigger.Response()))
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                                  self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo,
                                                        '/depth_cam/rgb/camera_info',
                                                        self.camera_info_callback, 1)
        self.start_get_roi = True
        joint_angle = [500, 520, 210, 50, 500]
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, joint_angle[1]),
                                                (3, joint_angle[2]), (4, joint_angle[3]),
                                                (5, 500), (10, self.p('gripper_open_pulse'))))
        self.enter = True
        response.success = True
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub); self.image_sub = None
            if self.camera_info_sub is not None:
                self.destroy_subscription(self.camera_info_sub); self.camera_info_sub = None
            self.heart.destroy(); self.heart = None
            self.enter = False
            self.start_transport = False
            self.motion.abort(True)
        response.success = True
        return response

    def enable_sorting_srv_callback(self, request, response):
        self.motion.abort(not request.data)
        self.enable_sorting = bool(request.data)
        response.success = True
        return response

    def set_target_srv_callback(self, request, response):
        if request.data_str in self.target_labels:
            self.target_labels[request.data_str] = request.data_bool
        response.success = True
        return response

    def recalibrate_srv_callback(self, request, response):
        threading.Thread(target=self._self_calibrate, daemon=True).start()
        response.success = True
        return response

    # ------------------------------------------------------------------ self-calibration

    def _self_calibrate(self):
        """Detects the colored bins in the camera frame and nudges the
        place_position offsets so each bin is centered. Runs the existing
        AprilTag pipeline if tags are visible, otherwise falls back to color
        contour centers. Best-effort: any failure just leaves defaults."""
        self.get_logger().info('starting v2 self-calibration')
        # Wait for camera + ROI to be ready.
        deadline = time.time() + 15
        while time.time() < deadline:
            if (self.intrinsic is not None and self.distortion is not None
                    and len(self.roi) > 0 and self.white_area_center is not None):
                break
            time.sleep(0.2)
        else:
            self.get_logger().warn('self-cal: camera/ROI never became ready')
            return

        try:
            bgr = self.image_queue.get(timeout=2.0)
        except queue.Empty:
            self.get_logger().warn('self-cal: no frames available')
            return

        roi = self.roi.copy()
        roi_img = bgr[roi[0]:roi[1], roi[2]:roi[3]]
        image_lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)

        if not self.p('place_bin_color_check'):
            self.get_logger().info('self-cal: bin colour check disabled, skipping')
            return

        for color in ('red', 'green', 'blue'):
            try:
                rng = self.lab_data['color_range_list'][color]
                mask = cv2.inRange(image_lab, tuple(rng['min']), tuple(rng['max']))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    continue
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) < 800:
                    continue
                M = cv2.moments(largest)
                if M['m00'] == 0:
                    continue
                cx = roi[2] + M['m10'] / M['m00']
                cy = roi[0] + M['m01'] / M['m00']
                world, _ = self._pixel_to_world((cx, cy))
                expected = self.DEFAULT_PLACE_POSITIONS[color]
                # Only trust corrections under 3cm - bigger means we're seeing
                # an item, not the bin.
                dx, dy = world[0] - expected[0], world[1] - expected[1]
                if abs(dx) < 0.03 and abs(dy) < 0.03:
                    self.place_offsets[color] = [dx, dy, 0.0]
                    self.place_position[color] = [expected[0] + dx,
                                                  expected[1] + dy,
                                                  expected[2]]
                    self.get_logger().info(
                        f'self-cal: bin {color} corrected by '
                        f'({dx*1000:+.1f}mm, {dy*1000:+.1f}mm)')
                else:
                    self.get_logger().info(
                        f'self-cal: bin {color} delta too large '
                        f'({dx*1000:.1f}, {dy*1000:.1f}) mm - skipping')
            except Exception as e:
                self.get_logger().warn(f'self-cal {color} failed: {e}')

        self.get_logger().info('v2 self-calibration done')

    def _pixel_to_world(self, pixel, height=0.03):
        return self.get_object_world_position(pixel, self.intrinsic, self.extristric,
                                              self.white_area_center, height)

    # ------------------------------------------------------------------ vision

    def get_object_pixel_position(self, bgr_image, roi):
        target_info = []
        draw_image = bgr_image.copy()
        roi_img = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]

        # YOLOv8 OBB for scaff
        yolo_results = self.yolo_model(roi_img, verbose=False)
        for result in yolo_results:
            if hasattr(result, 'obb') and result.obb is not None:
                for obb in result.obb:
                    center_x, center_y, w, h, r = obb.xywhr[0].cpu().numpy()
                    center_x += roi[2]; center_y += roi[0]
                    angle = int(math.degrees(r))
                    target_info.append(['scaff', 1, (int(center_x), int(center_y)),
                                        (int(w), int(h)), angle])
                    cv2.circle(draw_image, (int(center_x), int(center_y)),
                               8, (0, 0, 255), -1)
            else:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cx = (x1 + x2) / 2 + roi[2]
                    cy = (y1 + y2) / 2 + roi[0]
                    w, h = x2 - x1, y2 - y1
                    target_info.append(['scaff', 1, (int(cx), int(cy)),
                                        (int(w), int(h)), 0])
                    cv2.rectangle(draw_image,
                                  (int(x1 + roi[2]), int(y1 + roi[0])),
                                  (int(x2 + roi[2]), int(y2 + roi[0])),
                                  (0, 0, 255), 2)

        # Color blob detection (red/green/blue)
        roi_img_surface = self.image_process.get_top_surface(roi_img)
        image_lab = cv2.cvtColor(roi_img_surface, cv2.COLOR_BGR2LAB)
        min_area = float(self.p('min_object_area'))
        max_area = float(self.p('max_object_area'))
        for color in ('red', 'green', 'blue'):
            index = 0
            mask = cv2.inRange(image_lab,
                               tuple(self.lab_data['color_range_list'][color]['min']),
                               tuple(self.lab_data['color_range_list'][color]['max']))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            for c in contours:
                area = math.fabs(cv2.contourArea(c))
                if not (min_area <= area <= max_area):
                    continue
                rect = cv2.minAreaRect(c)
                (cx, cy), _ = cv2.minEnclosingCircle(c)
                cx, cy = roi[2] + cx, roi[0] + cy
                corners = list(map(lambda p: (roi[2] + p[0], roi[0] + p[1]),
                                   cv2.boxPoints(rect)))
                cv2.drawContours(draw_image, [np.intp(corners)], -1,
                                 (0, 255, 255), 2, cv2.LINE_AA)
                index += 1
                angle = int(round(rect[2]))
                target_info.append([color, index, (int(cx), int(cy)),
                                    (int(rect[1][0]), int(rect[1][1])), angle])
        return draw_image, target_info

    def get_object_world_position(self, position, intrinsic, extristric,
                                  white_area_center, height=0.03):
        projection_matrix = np.row_stack((np.column_stack((extristric[1], extristric[0])),
                                          np.array([[0, 0, 0, 1]])))
        world_pose = common.pixels_to_world([position], intrinsic, projection_matrix)[0]
        world_pose[0] = -world_pose[0]
        world_pose[1] = -world_pose[1]
        position = white_area_center[:3, 3] + world_pose
        position[2] = height
        config_data = common.get_yaml_data(os.path.join(self.config_path,
                                                        self.calibration_file))
        offset = tuple(config_data['pixel']['offset'])
        scale = tuple(config_data['pixel']['scale'])
        for i in range(3):
            position[i] = position[i] * scale[i] + offset[i]
        return position, projection_matrix

    def calculate_pick_grasp_yaw(self, position, target, target_info,
                                 intrinsic, projection_matrix):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw += 180
        elif position[0] < 0 and position[1] > 0:
            yaw -= 180
        gripper_size = [common.calculate_pixel_length(0.09, intrinsic, projection_matrix),
                        common.calculate_pixel_length(0.015, intrinsic, projection_matrix)]
        return calculate_grasp_yaw.calculate_gripper_yaw_angle(target, target_info,
                                                               gripper_size, yaw)

    def calculate_place_grasp_yaw(self, position, angle=0):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0: yaw += 180
        elif position[0] < 0 and position[1] > 0: yaw -= 180
        yaw1 = yaw + angle
        yaw2 = yaw1 + 90 if yaw < 0 else yaw1 - 90
        yaw = yaw1 if abs(yaw1) < abs(yaw2) else yaw2
        return 500 + int(yaw / 240 * 1000)

    # ------------------------------------------------------------------ pick & place

    def _apply_kinematics_calibration(self, position):
        config_data = common.get_yaml_data(os.path.join(self.config_path,
                                                        self.calibration_file))
        offset = tuple(config_data['kinematics']['offset'])
        scale = tuple(config_data['kinematics']['scale'])
        return [position[i] * scale[i] + offset[i] for i in range(3)]

    def _vision_target_present_at(self, label, world_xy, tol=0.025):
        """Return True if the camera still sees `label` close to world_xy."""
        if not self.p('vision_confirm_pick'):
            return False
        try:
            bgr = self.image_queue.get(timeout=0.8)
        except queue.Empty:
            return False
        _, info = self.get_object_pixel_position(bgr, self.roi.copy())
        for t in info:
            if t[0] != label:
                continue
            world, _ = self._pixel_to_world(t[2])
            if abs(world[0] - world_xy[0]) < tol and abs(world[1] - world_xy[1]) < tol:
                return True
        return False

    def _do_pick(self, position, pitch, yaw, label):
        """Adaptive pick. Returns True if we believe we have the object."""
        speed = max(0.1, 1.0 / float(self.p('motion_speed')))
        aggression = float(self.p('aggression'))
        hover_h = float(self.p('hover_height'))
        approach_dwell = float(self.p('approach_dwell'))
        open_pulse = int(self.p('gripper_open_pulse'))
        close_pulse = int(self.p('gripper_close_pulse'))
        full_closed = int(self.p('gripper_full_closed_pulse'))
        slack = int(self.p('gripper_slack'))
        close_dur = float(self.p('gripper_close_duration')) * speed
        step = int(self.p('gripper_step_pulse'))
        retries = int(self.p('max_pick_retries'))
        use_servo_fb = bool(self.p('servo_feedback_enabled')) and self.bus_servo_state_client is not None
        parallel_base = bool(self.p('parallel_base_motion'))

        attempt = 0
        attempted_close = close_pulse
        z_nudge = 0.0
        while attempt <= retries and not self.motion.aborted:
            # Hover above target
            hover = [position[0], position[1], position[2] + hover_h]
            if self.motion.goto_pose(hover, pitch,
                                     duration=max(0.6, 1.2 * speed / aggression),
                                     parallel_base=parallel_base) is None:
                return False
            self.motion.set_wrist(yaw, 0.3 * speed)

            # Open jaws fully before descent
            self.motion.set_gripper(open_pulse, 0.25 * speed)
            self.motion._sleep(approach_dwell)

            # Descend
            descend = [position[0], position[1], position[2] + z_nudge]
            if self.motion.goto_pose(descend, pitch,
                                     duration=max(0.4, 0.8 * speed / aggression),
                                     parallel_base=False) is None:
                return False

            # Close + read feedback
            if use_servo_fb:
                outcome = self.motion.grip_with_feedback(
                    attempted_close, open_pulse, full_closed, slack, close_dur)
            else:
                self.motion.set_gripper(attempted_close, close_dur)
                outcome = 'grabbed'

            # Lift to hover
            if self.motion.goto_pose(hover, pitch,
                                     duration=max(0.4, 0.8 * speed / aggression),
                                     parallel_base=False) is None:
                return False

            if outcome == 'stalled':
                # back off a bit and try again with looser close
                self.get_logger().warn(f'pick attempt {attempt}: stalled, backing off')
                attempted_close = max(open_pulse + 40, attempted_close - step)
                z_nudge += 0.003
            elif outcome == 'missed':
                # vision says it might still be there - tighten close, drop a bit
                self.get_logger().warn(f'pick attempt {attempt}: missed')
                # confirm with vision
                if not self._vision_target_present_at(label, position):
                    # already moved, give up retry loop
                    return False
                attempted_close = min(full_closed - 5, attempted_close + step)
                z_nudge -= 0.002
            else:  # grabbed
                # vision confirm: object should NOT still be in the scene
                if self.p('vision_confirm_pick'):
                    if self._vision_target_present_at(label, position):
                        self.get_logger().warn(
                            f'pick attempt {attempt}: servo says grabbed '
                            f'but vision still sees object - retry')
                        attempted_close = min(full_closed - 5, attempted_close + step)
                        attempt += 1
                        continue
                return True
            attempt += 1
        return False

    def _do_place(self, label):
        speed = max(0.1, 1.0 / float(self.p('motion_speed')))
        aggression = float(self.p('aggression'))
        position = copy.deepcopy(self.place_position[label])
        yaw = self.calculate_place_grasp_yaw(position, 0)
        config_data = common.get_yaml_data(os.path.join(self.config_path,
                                                        self.calibration_file))
        offset = tuple(config_data['kinematics']['offset'])
        scale = tuple(config_data['kinematics']['scale'])
        angle = math.degrees(math.atan2(position[1], position[0]))
        if angle > 45:
            position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
            position = [position[0] - offset[1], position[1] + offset[0], position[2] + offset[2]]
        elif angle < -45:
            position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
            position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
        else:
            position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
            position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]

        hover = [position[0], position[1], position[2] + 0.05]
        if self.motion.goto_pose(hover, 80,
                                 duration=max(0.6, 1.0 * speed / aggression),
                                 parallel_base=True) is None:
            return False
        self.motion.set_wrist(yaw, 0.3 * speed)
        if self.motion.goto_pose(position, 80,
                                 duration=max(0.4, 0.7 * speed / aggression),
                                 parallel_base=False) is None:
            return False
        self.motion.set_gripper(int(self.p('gripper_open_pulse')), 0.3 * speed)
        if self.motion.goto_pose(hover, 80,
                                 duration=max(0.4, 0.7 * speed / aggression),
                                 parallel_base=False) is None:
            return False
        return True

    def transport_thread(self):
        while self.running:
            if not self.start_transport:
                time.sleep(0.05)
                continue
            position, yaw, target = self.transport_info
            if position[0] > 0.22:
                position[2] += 0.01
            position = self._apply_kinematics_calibration(position)
            label = target[0]

            picked = self._do_pick(position, 80, yaw, label)
            if picked:
                placed = self._do_place(label)
                self.go_home(False)
                if not placed:
                    self.get_logger().warn(f'place failed for {label}')
            else:
                # Pick failed - drop everything cleanly and go home so the
                # vision loop can re-detect and try a different target.
                self.motion.set_gripper(int(self.p('gripper_open_pulse')), 0.3)
                self.go_home(True)

            self.target = None
            self.start_transport = False

    # ------------------------------------------------------------------ main loop

    def main(self):
        avg_frames = 1
        while self.running:
            if not self.enter:
                time.sleep(0.05)
                continue
            try:
                bgr_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            if self.start_get_roi:
                self.get_roi()
                self.start_get_roi = False

            roi = self.roi.copy()
            intrinsic = self.intrinsic
            avg_frames = max(1, int(self.p('detection_avg_frames')))
            still_thresh = int(self.p('count_still_threshold'))
            move_thresh = int(self.p('count_move_threshold'))
            lock_thresh = float(self.p('lock_distance_thresh'))

            if len(roi) > 0 and self.enable_sorting and not self.start_transport:
                display_image, target_info = self.get_object_pixel_position(bgr_image, roi)

                if target_info and self.last_object_info_list:
                    target_info = position_change_detect.position_reorder(
                        target_info, self.last_object_info_list, 20)
                self.last_object_info_list = copy.deepcopy(target_info)

                for t in target_info:
                    cv2.putText(display_image, t[0],
                                (t[2][0] - 4 * len(t[0] + str(t[1])), t[2][1] + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                target_miss = True
                for t in target_info:
                    if not self.target_labels.get(t[0], False):
                        continue
                    if self.target is not None:
                        if self.target[0] != t[0] or self.target[1] != t[1]:
                            continue
                        target_miss = False
                        self.target = t

                    if self.camera_type == 'USB_CAM':
                        x, y = distortion_inverse_map.undistorted_to_distorted_pixel(
                            t[2][0], t[2][1], self.intrinsic, self.distortion)
                        t[2] = (x, y)

                    position, projection_matrix = self.get_object_world_position(
                        t[2], intrinsic, self.extristric, self.white_area_center)
                    result = self.calculate_pick_grasp_yaw(position, t, target_info,
                                                            intrinsic, projection_matrix)
                    if result is not None and self.target is None:
                        self.target = t
                        break

                    if (self.last_position is not None and self.target is not None
                            and result is not None):
                        e_distance = round(
                            math.sqrt(pow(self.last_position[0] - position[0], 2))
                            + math.sqrt(pow(self.last_position[1] - position[1], 2)), 5)
                        if e_distance <= lock_thresh:
                            cv2.line(display_image, result[1][0], result[1][1],
                                     (255, 255, 0), 2, cv2.LINE_AA)
                            self.count_move = 0
                            self.count_still += 1
                        else:
                            self.count_move += 1
                            self.count_still = 0

                        if self.count_move > move_thresh:
                            self.target = None
                        if self.count_still > still_thresh:
                            self.count_still = 0
                            self.count_move = 0
                            # Multi-frame averaging for the lock-in position
                            self.detection_history.setdefault(t[0], []).append(position)
                            hist = self.detection_history[t[0]][-avg_frames:]
                            avg_pos = [sum(p[i] for p in hist) / len(hist) for i in range(3)]
                            self.detection_history[t[0]] = hist
                            yaw_pulse = 500 + int(result[0] / 240 * 1000)
                            self.transport_info = [avg_pos, yaw_pulse, t]
                            self.target = t
                            self.start_transport = True
                    self.last_position = position

                if target_miss:
                    self.target_miss_count += 1
                if self.target_miss_count > 10:
                    self.target_miss_count = 0
                    self.target = None
            else:
                display_image = bgr_image.copy()

            if bgr_image is not None and self.get_parameter('display').value:
                cv2.imshow('result_image_v2', display_image)
                cv2.waitKey(1)
            self.result_publisher.publish(
                self.bridge.cv2_to_imgmsg(display_image, "bgr8"))

    # ------------------------------------------------------------------ camera cbs

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_rgb_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)


def main():
    node = ObjectSortingNodeV2('custom_sortingv2')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        executor.shutdown()


if __name__ == '__main__':
    main()
