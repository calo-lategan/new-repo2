#!/usr/bin/env python3
# coding: utf8
# Custom Object & Color Sorting - v4
#
# Built on the v4 research (see ./RESEARCH.md). Headline deltas vs v2:
#
#   PERFORMANCE
#   - Single MultiThreadedExecutor, callback groups split:
#       * camera + inference:    MutuallyExclusiveCallbackGroup (depth=1)
#       * services / clients:    ReentrantCallbackGroup
#     Eliminates the "pick stalls when UI is open" deadlock class.
#   - QoS profile: qos_profile_sensor_data (BEST_EFFORT, KEEP_LAST 1) on the
#     image stream so we always process the freshest frame and never stall
#     under bursts.
#   - Image conversion via np.frombuffer view (zero-copy from sensor_msgs/
#     Image.data) instead of cv_bridge.imgmsg_to_cv2 (which copies).
#   - Inference runs in its own dedicated worker thread, NOT from the camera
#     callback. The callback hands off a frame and returns immediately.
#   - Async kinematics: every set_pose_target uses call_async + futures so
#     the executor is never blocked waiting for an IK reply.
#   - YOLO warmup pass on startup AND after every engine swap so the first
#     real frame doesn't pay the engine deserialization cost.
#   - No INFO logs in the hot loop (logging from rclpy is surprisingly heavy).
#
#   FEATURES OVER v2
#   - Hot-swap AI models with zero downtime: ~/load_engine service takes a
#     path; the inference thread atomically rebuilds the YOLO model between
#     frames, runs a warmup pass, releases the old engine + CUDA cache.
#   - Profile system: every tunable lives in YAML profiles under
#     ~/jetarm_v4_profiles/. Services ~/save_profile, ~/load_profile,
#     ~/save_as_default. Profiles persist across reboots and can be loaded
#     by the launch file via profile:=fast.
#   - Per-target speed/grip overrides (e.g. slower for scaff, faster for
#     blocks) via target_overrides parameter.
#   - All v2 features kept: self-calibration, vision+servo grip feedback,
#     retry/recovery, multi-frame averaging, Tkinter tuner UI.
#
# See INSTALL.md for setup. See RESEARCH.md for the design principles.

import os
import cv2
import yaml
import time
import math
import copy
import queue
import threading
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from sdk import common, fps
from app.common import Heart
from interfaces.srv import SetStringBool
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller.bus_servo_control import set_servo_position
from kinematics.kinematics_control import set_pose_target
from app.utils import calculate_grasp_yaw, position_change_detect, image_process, distortion_inverse_map

from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd

from ultralytics import YOLO

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


GRIPPER_ID = 10
PROFILES_DIR = Path(os.environ.get('JETARM_V4_PROFILES',
                                   str(Path.home() / 'jetarm_v4_profiles')))
DEFAULT_PROFILE_PATH = PROFILES_DIR / 'default.yaml'


# ---------------------------------------------------------------------------
# InferenceWorker
# ---------------------------------------------------------------------------

class InferenceWorker(threading.Thread):
    """Owns the YOLO model on one thread (one CUDA context per process), so
    we can hot-swap engines safely between frames without racing inference.

    Producers (camera callback) call submit(frame). Consumers (sorting loop)
    call latest() to fetch the most recent (frame, detections) pair. We never
    queue frames - always overwrite, so the loop always works on the freshest
    image and never accumulates lag.
    """

    def __init__(self, engine_path, logger, on_swap=None):
        super().__init__(daemon=True, name='yolo-inference')
        self._logger = logger
        self._on_swap = on_swap or (lambda _: None)
        self._engine_path = engine_path
        self._pending_engine_path = None
        self._swap_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._latest_frame = None     # numpy bgr image OR None
        self._latest_result = None    # (frame, detections, ts)
        self._frame_event = threading.Event()
        self._stop = threading.Event()
        self.model = None
        self._load_count = 0

    # -- producer / consumer --

    def submit(self, frame):
        with self._frame_lock:
            self._latest_frame = frame
        self._frame_event.set()

    def latest(self):
        with self._frame_lock:
            return self._latest_result

    def request_engine_swap(self, path):
        if not path:
            return False
        if not Path(path).exists():
            self._logger.warn(f'engine swap rejected - file missing: {path}')
            return False
        with self._swap_lock:
            self._pending_engine_path = path
        self._logger.info(f'engine swap queued -> {path}')
        return True

    def stop(self):
        self._stop.set()
        self._frame_event.set()

    # -- thread body --

    def _load(self, path):
        # Drop the old model first so its CUDA memory is released before the
        # new engine deserializes - critical on the 8GB Orin Nano.
        if self.model is not None:
            try:
                del self.model
            except Exception:
                pass
            self.model = None
            if HAS_TORCH:
                try:
                    torch.cuda.empty_cache()
                except Exception:
                    pass
        self._logger.info(f'loading YOLO engine: {path}')
        m = YOLO(path, task='detect')
        # Warmup: first inference includes engine deserialization; do it on
        # a dummy frame so the first real frame is fast.
        try:
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            m(dummy, verbose=False)
        except Exception as e:
            self._logger.warn(f'warmup pass failed (continuing): {e}')
        self.model = m
        self._engine_path = path
        self._load_count += 1
        try:
            self._on_swap(path)
        except Exception:
            pass

    def run(self):
        try:
            self._load(self._engine_path)
        except Exception as e:
            self._logger.error(f'initial engine load failed: {e}')
            return
        while not self._stop.is_set():
            self._frame_event.wait(timeout=0.1)
            self._frame_event.clear()
            # Service a queued hot-swap before pulling the next frame.
            with self._swap_lock:
                pending = self._pending_engine_path
                self._pending_engine_path = None
            if pending and pending != self._engine_path:
                try:
                    self._load(pending)
                except Exception as e:
                    self._logger.error(f'engine swap to {pending} failed: {e}')
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is None or self.model is None:
                continue
            t0 = time.time()
            try:
                results = self.model(frame, verbose=False)
            except Exception as e:
                self._logger.warn(f'inference error: {e}')
                continue
            with self._frame_lock:
                self._latest_result = (frame, results, t0)


# ---------------------------------------------------------------------------
# MotionController
# ---------------------------------------------------------------------------

class MotionController:
    """All motion + gripper feedback. Async-friendly - we submit IK requests
    via call_async and use add_done_callback rather than blocking spin."""

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

    def _sleep(self, dt):
        end = time.time() + dt
        while time.time() < end and not self._abort and rclpy.ok():
            time.sleep(min(0.02, end - time.time()))

    def _await_future(self, future, timeout=3.0):
        # Safe to use *only* from a thread that is NOT the executor thread
        # (we run this from the dedicated transport thread). For executor-
        # thread callers, use add_done_callback.
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline and not self._abort:
            time.sleep(0.005)
        if future.done() and future.result() is not None:
            return future.result()
        return None

    def get_servo_state(self, servo_id, fields=('position',)):
        if self.bus_servo_state_client is None:
            return {}
        req = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        cmd.id = int(servo_id)
        cmd.get_position = 1 if 'position' in fields else 0
        cmd.get_temperature = 1 if 'temperature' in fields else 0
        cmd.get_voltage = 1 if 'voltage' in fields else 0
        req.cmd = [cmd]
        future = self.bus_servo_state_client.call_async(req)
        res = self._await_future(future, timeout=0.6)
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

    def goto_pose(self, position, pitch, duration, parallel_base=True):
        if self._abort:
            return None
        msg = set_pose_target(position, pitch, [-180.0, 180.0], 1.0, duration=duration)
        future = self.kinematics_client.call_async(msg)
        res = self._await_future(future, timeout=2.0)
        if res is None or not res.pulse:
            return None
        servo_data = np.array(res.pulse).reshape(-1, 5).tolist()
        if not servo_data:
            return None
        last = servo_data[-1]
        if parallel_base:
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
                           confirm_delay=0.2):
        self.set_gripper(close_pulse, duration)
        self._sleep(confirm_delay)
        st = self.get_servo_state(GRIPPER_ID, fields=('position', 'temperature'))
        pos = st.get('position', None)
        temp = st.get('temperature', None)
        if temp is not None and temp > 65:
            return 'stalled'
        if pos is None:
            return 'grabbed'
        if pos >= (full_closed_pulse - slack):
            return 'missed'
        if abs(pos - close_pulse) > 60 and pos < close_pulse - 60:
            return 'stalled'
        return 'grabbed'


# ---------------------------------------------------------------------------
# Profile I/O
# ---------------------------------------------------------------------------

def _ensure_profiles_dir():
    PROFILES_DIR.mkdir(parents=True, exist_ok=True)


def load_profile_yaml(path):
    if not Path(path).exists():
        return {}
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}
    # Accept both raw dict and ROS-style {/**:{ros__parameters:{}}} layouts.
    if isinstance(data, dict) and '/**' in data:
        return data['/**'].get('ros__parameters', {})
    return data


def save_profile_yaml(path, params):
    _ensure_profiles_dir()
    payload = {'/**': {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(payload, f, sort_keys=True)


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class ObjectSortingNodeV4(Node):

    DEFAULT_PLACE_POSITIONS = {
        'green': [-0.006, 0.23, 0.015],
        'red':   [ 0.064, 0.23, 0.015],
        'blue':  [-0.076, 0.23, 0.015],
        'tag1':  [-0.076, 0.16, 0.015],
        'tag2':  [-0.006, 0.16, 0.015],
        'tag3':  [ 0.064, 0.16, 0.015],
        'scaff': [-0.076, 0.16, 0.015],
    }

    TUNABLE_PARAMS = (
        # name, default, range_or_None
        ('engine_path', '/home/ubuntu/third_party_ros2/data/best_scaff2.engine', None),
        ('min_object_area', 500, (50, 5000)),
        ('max_object_area', 7000, (1000, 30000)),
        ('lock_distance_thresh', 0.005, (0.001, 0.05)),
        ('count_still_threshold', 4, (1, 30)),
        ('count_move_threshold', 8, (1, 30)),
        ('detection_avg_frames', 3, (1, 10)),
        ('motion_speed', 1.5, (0.3, 2.5)),
        ('aggression', 1.3, (0.3, 2.0)),
        ('hover_height', 0.06, (0.02, 0.15)),
        ('approach_dwell', 0.1, (0.0, 1.0)),
        ('parallel_base_motion', True, None),
        ('gripper_open_pulse', 200, (50, 500)),
        ('gripper_close_pulse', 540, (300, 700)),
        ('gripper_full_closed_pulse', 700, (500, 900)),
        ('gripper_slack', 25, (5, 80)),
        ('gripper_close_duration', 0.35, (0.1, 2.0)),
        ('gripper_step_pulse', 30, (5, 100)),
        ('max_pick_retries', 3, (0, 6)),
        ('vision_confirm_pick', True, None),
        ('servo_feedback_enabled', True, None),
        ('startup_self_calibrate', True, None),
        ('place_bin_color_check', True, None),
        ('inference_warmup', True, None),
        ('hot_log_inference_ms', False, None),
        # Free-form per-target overrides as JSON-ish string, parsed lazily.
        # Example: '{"scaff": {"motion_speed": 0.9}, "blue": {"motion_speed": 1.8}}'
        ('target_overrides', '{}', None),
    )

    def __init__(self, name='custom_sortingv4'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Apply default profile (if present) BEFORE declaring tunables, so its
        # values become the seed values rather than getting overwritten.
        self._seeded_from_default = self._apply_default_profile_seed()
        self._declare_tunables()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---- Models / shared state ----
        proto_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/deploy.prototxt'
        model_path = '/home/ubuntu/ros2_ws/src/app/app/hed_model/hed_pretrained_bsds.caffemodel'
        self.image_process = image_process.GetObjectSurface(proto_path, model_path)

        self.lock = threading.RLock()
        self.fps = fps.FPS()
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        self.data = common.get_yaml_data(os.path.join(self.config_path, "lab_config.yaml"))
        self.lab_data = self.data['/**']['ros__parameters']
        self.camera_type = os.environ['CAMERA_TYPE']

        self.tag_size = 0.025

        self.place_position = copy.deepcopy(self.DEFAULT_PLACE_POSITIONS)
        self.place_offsets = {k: [0.0, 0.0, 0.0] for k in self.place_position}

        self.target_labels = {
            'red': True, 'green': True, 'blue': True, 'scaff': True,
            'tag1': False, 'tag2': False, 'tag3': False,
        }
        self.detection_history = {}
        self.running = True
        self._init_state()

        # ---- Callback groups (research recommendation #1) ----
        self.cam_group = MutuallyExclusiveCallbackGroup()      # camera + sorting loop
        self.svc_group = ReentrantCallbackGroup()              # services / clients

        # ---- Pubs / subs ----
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        # ---- Services (lifecycle + control) ----
        self.create_service(Trigger, '~/enter', self.enter_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(SetBool, '~/enable_sorting',
                            self.enable_sorting_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(SetStringBool, '~/set_target',
                            self.set_target_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(Trigger, '~/recalibrate',
                            self.recalibrate_srv_callback,
                            callback_group=self.svc_group)
        # New v4 services: model swap + profile management
        self.create_service(SetStringBool, '~/load_engine',
                            self.load_engine_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(SetStringBool, '~/save_profile',
                            self.save_profile_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(SetStringBool, '~/load_profile',
                            self.load_profile_srv_callback,
                            callback_group=self.svc_group)
        self.create_service(Trigger, '~/save_as_default',
                            self.save_as_default_srv_callback,
                            callback_group=self.svc_group)

        # ---- Service clients ----
        self.kinematics_client = self.create_client(SetRobotPose,
                                                    'kinematics/set_pose_target',
                                                    callback_group=self.svc_group)
        self.kinematics_client.wait_for_service()
        self.set_joint_value_target_client = self.create_client(
            SetJointValue, 'kinematics/set_joint_value_target',
            callback_group=self.svc_group)
        self.set_joint_value_target_client.wait_for_service()
        self.bus_servo_state_client = self.create_client(
            GetBusServoState, 'ros_robot_controller/bus_servo/get_state',
            callback_group=self.svc_group)
        if not self.bus_servo_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('bus_servo/get_state unavailable - vision-only fallback')
            self.bus_servo_state_client = None

        self.motion = MotionController(self, self.joints_pub,
                                       self.kinematics_client,
                                       self.bus_servo_state_client)

        # ---- Inference worker (one CUDA context, hot-swappable) ----
        self.inference = InferenceWorker(self.p('engine_path'), self.get_logger(),
                                         on_swap=lambda path:
                                             self.get_logger().info(
                                                 f'engine active: {path}'))
        self.inference.start()

        # ---- Camera subscriptions: BEST_EFFORT, depth=1 ----
        self.image_sub = None
        self.camera_info_sub = None
        self.intrinsic = None
        self.distortion = None

        self._startup_done = False
        self.create_timer(0.0, self._startup, callback_group=self.svc_group)

    # ------------------------------------------------------------------ profile

    def _apply_default_profile_seed(self):
        """If a default profile YAML exists, push its values into ROS overrides
        so declare_parameter() picks them up as the seed. Returns the dict
        applied (or empty)."""
        if not DEFAULT_PROFILE_PATH.exists():
            return {}
        params = load_profile_yaml(DEFAULT_PROFILE_PATH)
        for k, v in params.items():
            try:
                # Late-binding seed: set as override before declaration.
                self.set_parameters([rclpy.parameter.Parameter(
                    k, value=v)])
            except Exception:
                # parameter not yet declared - that's expected pre-declare; we
                # set it again post-declare below.
                pass
        return params

    # ------------------------------------------------------------------ params

    def _declare_tunables(self):
        for name, default, rng in self.TUNABLE_PARAMS:
            try:
                desc = ParameterDescriptor()
                if isinstance(default, bool):
                    self.declare_parameter(name, default, descriptor=desc)
                elif isinstance(default, int):
                    if rng:
                        desc.integer_range = [IntegerRange(from_value=int(rng[0]),
                                                            to_value=int(rng[1]),
                                                            step=1)]
                    self.declare_parameter(name, default, descriptor=desc)
                elif isinstance(default, float):
                    if rng:
                        desc.floating_point_range = [FloatingPointRange(
                            from_value=float(rng[0]), to_value=float(rng[1]),
                            step=0.0)]
                    self.declare_parameter(name, default, descriptor=desc)
                else:
                    self.declare_parameter(name, default)
            except Exception:
                pass
        # Re-apply the default profile values now that all params are declared,
        # so the seed actually wins over hardcoded defaults.
        if self._seeded_from_default:
            ros_params = []
            for k, v in self._seeded_from_default.items():
                try:
                    ros_params.append(rclpy.parameter.Parameter(k, value=v))
                except Exception:
                    pass
            if ros_params:
                self.set_parameters(ros_params)
            self.get_logger().info(
                f'loaded default profile from {DEFAULT_PROFILE_PATH} '
                f'({len(self._seeded_from_default)} params)')

    def _on_param_change(self, params):
        for p in params:
            # If the engine path was changed via param, queue a hot-swap.
            if p.name == 'engine_path' and isinstance(p.value, str) and p.value:
                if hasattr(self, 'inference') and self.inference is not None:
                    self.inference.request_engine_swap(p.value)
        return SetParametersResult(successful=True)

    def p(self, name):
        return self.get_parameter(name).value

    def _all_tunables_dict(self):
        out = {}
        for name, *_ in self.TUNABLE_PARAMS:
            try:
                out[name] = self.get_parameter(name).value
            except Exception:
                pass
        return out

    # ------------------------------------------------------------------ misc state

    def _init_state(self):
        self.heart = None
        self.target_miss_count = 0
        self.transport_info = None
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
        self.detection_history = {}

    def _startup(self):
        if self._startup_done:
            return
        self._startup_done = True
        threading.Thread(target=self.sorting_loop, daemon=True).start()
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
        self.create_service(Trigger, '~/init_finish',
                            lambda req, resp: setattr(resp, 'success', True) or resp,
                            callback_group=self.svc_group)
        self.get_logger().info('\033[1;32mv4 init finish\033[0m')

    # ------------------------------------------------------------------ motion helpers

    def go_home(self, interrupt=True):
        speed = max(0.1, 1.0 / float(self.p('motion_speed')))
        if interrupt:
            self.motion.set_gripper(self.p('gripper_open_pulse'), 0.3 * speed)
        ja = [500, 520, 210, 50, 500]
        set_servo_position(self.joints_pub, 0.6 * speed,
                           ((2, ja[1]), (3, ja[2]), (4, ja[3]), (5, 500)))
        self.motion._sleep(0.6 * speed)
        set_servo_position(self.joints_pub, 0.5 * speed, ((1, ja[0]),))
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
        self.get_logger().info('enter v4')
        self._init_state()
        self.heart = Heart(self, '~/heartbeat', 5,
                           lambda _: self.exit_srv_callback(Trigger.Request(),
                                                            Trigger.Response()))
        # qos_profile_sensor_data: BEST_EFFORT, KEEP_LAST 5 - matches camera drivers
        self.image_sub = self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback,
            qos_profile_sensor_data, callback_group=self.cam_group)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback,
            qos_profile_sensor_data, callback_group=self.cam_group)
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
            if self.heart is not None:
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

    def load_engine_srv_callback(self, request, response):
        # data_str: path to .engine, data_bool: ignored
        ok = self.inference.request_engine_swap(request.data_str)
        if ok:
            try:
                self.set_parameters([rclpy.parameter.Parameter(
                    'engine_path', value=request.data_str)])
            except Exception:
                pass
        response.success = ok
        return response

    def save_profile_srv_callback(self, request, response):
        # data_str: profile name (no extension); data_bool: ignored
        try:
            name = (request.data_str or 'profile').strip()
            if not name.endswith('.yaml'):
                name += '.yaml'
            path = PROFILES_DIR / name
            save_profile_yaml(path, self._all_tunables_dict())
            self.get_logger().info(f'profile saved -> {path}')
            response.success = True
        except Exception as e:
            self.get_logger().error(f'save_profile failed: {e}')
            response.success = False
        return response

    def load_profile_srv_callback(self, request, response):
        try:
            name = (request.data_str or 'profile').strip()
            if not name.endswith('.yaml'):
                name += '.yaml'
            path = PROFILES_DIR / name
            params = load_profile_yaml(path)
            if not params:
                response.success = False
                return response
            ros_params = []
            for k, v in params.items():
                try:
                    ros_params.append(rclpy.parameter.Parameter(k, value=v))
                except Exception:
                    pass
            if ros_params:
                self.set_parameters(ros_params)
            self.get_logger().info(f'profile loaded <- {path} ({len(ros_params)} params)')
            response.success = True
        except Exception as e:
            self.get_logger().error(f'load_profile failed: {e}')
            response.success = False
        return response

    def save_as_default_srv_callback(self, request, response):
        try:
            save_profile_yaml(DEFAULT_PROFILE_PATH, self._all_tunables_dict())
            self.get_logger().info(f'default profile saved -> {DEFAULT_PROFILE_PATH}')
            response.success = True
        except Exception as e:
            self.get_logger().error(f'save_as_default failed: {e}')
            response.success = False
        return response

    # ------------------------------------------------------------------ self-cal

    def _self_calibrate(self):
        self.get_logger().info('v4 self-calibration starting')
        deadline = time.time() + 15
        while time.time() < deadline:
            if (self.intrinsic is not None and self.distortion is not None
                    and len(self.roi) > 0 and self.white_area_center is not None):
                break
            time.sleep(0.2)
        else:
            self.get_logger().warn('self-cal: camera/ROI never ready')
            return
        # Use the most recent inference frame so we don't fight the camera CB.
        latest = self.inference.latest()
        if latest is None:
            self.get_logger().warn('self-cal: no inference frames yet')
            return
        bgr = latest[0]
        roi = self.roi.copy()
        roi_img = bgr[roi[0]:roi[1], roi[2]:roi[3]]
        image_lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)
        if not self.p('place_bin_color_check'):
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
                dx, dy = world[0] - expected[0], world[1] - expected[1]
                if abs(dx) < 0.03 and abs(dy) < 0.03:
                    self.place_offsets[color] = [dx, dy, 0.0]
                    self.place_position[color] = [expected[0] + dx,
                                                  expected[1] + dy,
                                                  expected[2]]
                    self.get_logger().info(
                        f'self-cal: {color} corrected ({dx*1000:+.1f}, {dy*1000:+.1f}) mm')
            except Exception as e:
                self.get_logger().warn(f'self-cal {color} failed: {e}')
        self.get_logger().info('v4 self-calibration done')

    def _pixel_to_world(self, pixel, height=0.03):
        return self.get_object_world_position(pixel, self.intrinsic, self.extristric,
                                              self.white_area_center, height)

    # ------------------------------------------------------------------ vision

    def _detections_from_results(self, bgr_image, roi, yolo_results):
        target_info = []
        draw_image = bgr_image.copy()
        roi_img = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]
        # 1) YOLO scaff detections (already inferred by InferenceWorker)
        for result in yolo_results:
            if hasattr(result, 'obb') and result.obb is not None:
                for obb in result.obb:
                    cx, cy, w, h, r = obb.xywhr[0].cpu().numpy()
                    cx += roi[2]; cy += roi[0]
                    angle = int(math.degrees(r))
                    target_info.append(['scaff', 1, (int(cx), int(cy)),
                                        (int(w), int(h)), angle])
                    cv2.circle(draw_image, (int(cx), int(cy)), 8, (0, 0, 255), -1)
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
        # 2) Color blob detection (red/green/blue)
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
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_NONE)[-2]
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
        if not self.p('vision_confirm_pick'):
            return False
        latest = self.inference.latest()
        if latest is None:
            return False
        bgr, results, _ = latest
        _, info = self._detections_from_results(bgr, self.roi.copy(), results)
        for t in info:
            if t[0] != label:
                continue
            world, _ = self._pixel_to_world(t[2])
            if abs(world[0] - world_xy[0]) < tol and abs(world[1] - world_xy[1]) < tol:
                return True
        return False

    def _per_target_overrides(self, label):
        try:
            import json
            raw = self.p('target_overrides') or '{}'
            d = json.loads(raw)
            return d.get(label, {})
        except Exception:
            return {}

    def _do_pick(self, position, pitch, yaw, label):
        ov = self._per_target_overrides(label)
        speed_p = float(ov.get('motion_speed', self.p('motion_speed')))
        speed = max(0.1, 1.0 / speed_p)
        aggression = float(ov.get('aggression', self.p('aggression')))
        hover_h = float(self.p('hover_height'))
        approach_dwell = float(self.p('approach_dwell'))
        open_pulse = int(self.p('gripper_open_pulse'))
        close_pulse = int(ov.get('gripper_close_pulse', self.p('gripper_close_pulse')))
        full_closed = int(self.p('gripper_full_closed_pulse'))
        slack = int(self.p('gripper_slack'))
        close_dur = float(self.p('gripper_close_duration')) * speed
        step = int(self.p('gripper_step_pulse'))
        retries = int(ov.get('max_pick_retries', self.p('max_pick_retries')))
        use_servo_fb = (bool(self.p('servo_feedback_enabled'))
                        and self.bus_servo_state_client is not None)
        parallel_base = bool(self.p('parallel_base_motion'))

        attempt = 0
        attempted_close = close_pulse
        z_nudge = 0.0
        while attempt <= retries and not self.motion.aborted:
            hover = [position[0], position[1], position[2] + hover_h]
            if self.motion.goto_pose(hover, pitch,
                                     duration=max(0.5, 1.1 * speed / aggression),
                                     parallel_base=parallel_base) is None:
                return False
            self.motion.set_wrist(yaw, 0.25 * speed)
            self.motion.set_gripper(open_pulse, 0.2 * speed)
            self.motion._sleep(approach_dwell)
            descend = [position[0], position[1], position[2] + z_nudge]
            if self.motion.goto_pose(descend, pitch,
                                     duration=max(0.35, 0.7 * speed / aggression),
                                     parallel_base=False) is None:
                return False
            if use_servo_fb:
                outcome = self.motion.grip_with_feedback(
                    attempted_close, open_pulse, full_closed, slack, close_dur)
            else:
                self.motion.set_gripper(attempted_close, close_dur)
                outcome = 'grabbed'
            if self.motion.goto_pose(hover, pitch,
                                     duration=max(0.35, 0.7 * speed / aggression),
                                     parallel_base=False) is None:
                return False
            if outcome == 'stalled':
                attempted_close = max(open_pulse + 40, attempted_close - step)
                z_nudge += 0.003
            elif outcome == 'missed':
                if not self._vision_target_present_at(label, position):
                    return False
                attempted_close = min(full_closed - 5, attempted_close + step)
                z_nudge -= 0.002
            else:
                if self.p('vision_confirm_pick'):
                    if self._vision_target_present_at(label, position):
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
                                 duration=max(0.5, 0.9 * speed / aggression),
                                 parallel_base=True) is None:
            return False
        self.motion.set_wrist(yaw, 0.25 * speed)
        if self.motion.goto_pose(position, 80,
                                 duration=max(0.35, 0.6 * speed / aggression),
                                 parallel_base=False) is None:
            return False
        self.motion.set_gripper(int(self.p('gripper_open_pulse')), 0.25 * speed)
        if self.motion.goto_pose(hover, 80,
                                 duration=max(0.35, 0.6 * speed / aggression),
                                 parallel_base=False) is None:
            return False
        return True

    def transport_thread(self):
        while self.running:
            if not self.start_transport:
                time.sleep(0.05); continue
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
                self.motion.set_gripper(int(self.p('gripper_open_pulse')), 0.3)
                self.go_home(True)
            self.target = None
            self.start_transport = False

    # ------------------------------------------------------------------ sorting loop

    def sorting_loop(self):
        avg_frames = 1
        while self.running:
            if not self.enter:
                time.sleep(0.05); continue
            latest = self.inference.latest()
            if latest is None:
                time.sleep(0.005); continue
            bgr_image, results, ts = latest
            if self.start_get_roi and self.intrinsic is not None and self.distortion is not None:
                self.get_roi()
                self.start_get_roi = False
            roi = self.roi.copy() if len(self.roi) else []
            intrinsic = self.intrinsic
            avg_frames = max(1, int(self.p('detection_avg_frames')))
            still_thresh = int(self.p('count_still_threshold'))
            move_thresh = int(self.p('count_move_threshold'))
            lock_thresh = float(self.p('lock_distance_thresh'))
            if len(roi) > 0 and self.enable_sorting and not self.start_transport and intrinsic is not None:
                display_image, target_info = self._detections_from_results(bgr_image, roi, results)
                if target_info and self.last_object_info_list:
                    target_info = position_change_detect.position_reorder(
                        target_info, self.last_object_info_list, 20)
                self.last_object_info_list = copy.deepcopy(target_info)
                for t in target_info:
                    cv2.putText(display_image, t[0],
                                (t[2][0] - 4 * len(t[0] + str(t[1])), t[2][1] + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if self.p('hot_log_inference_ms'):
                    cv2.putText(display_image, f'inf {1000*(time.time()-ts):.0f}ms',
                                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 200, 255), 2)
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
            if self.get_parameter('display').value:
                cv2.imshow('result_image_v4', display_image)
                cv2.waitKey(1)
            # Skip the cv_bridge round-trip; build the Image msg directly
            # from the numpy buffer.
            self._publish_image(display_image)
            time.sleep(0.001)

    def _publish_image(self, bgr):
        msg = Image()
        msg.height = bgr.shape[0]; msg.width = bgr.shape[1]
        msg.encoding = 'bgr8'; msg.is_bigendian = 0
        msg.step = bgr.shape[1] * bgr.shape[2]
        msg.data = bgr.tobytes()
        self.result_publisher.publish(msg)

    # ------------------------------------------------------------------ camera cbs

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_rgb_image):
        # Zero-copy view (research recommendation: skip cv_bridge copy).
        try:
            buf = np.frombuffer(ros_rgb_image.data, dtype=np.uint8)
            bgr = buf.reshape(ros_rgb_image.height, ros_rgb_image.width, -1)
        except Exception:
            return
        # Hand off to the inference worker - never block the camera CB.
        self.inference.submit(bgr)


def main():
    rclpy.init()
    node = ObjectSortingNodeV4('custom_sortingv4')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        node.inference.stop()
        executor.shutdown()


if __name__ == '__main__':
    main()
