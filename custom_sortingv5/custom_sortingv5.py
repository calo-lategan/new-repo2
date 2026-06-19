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
#   - QoS profile: 1 (BEST_EFFORT, KEEP_LAST 1) on the
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
#     ~/jetarm_v5_profiles/. Services ~/save_profile, ~/load_profile,
#     ~/save_as_default. Profiles persist across reboots and can be loaded
#     by the launch file via profile:=fast.
#   - Per-target speed/grip overrides (e.g. slower for scaff, faster for
#     blocks) via target_overrides parameter.
#   - All v2 features kept: self-calibration, vision+servo grip feedback,
#     retry/recovery, multi-frame averaging, Tkinter tuner UI.
#
#   DEBUG / DIAGNOSTICS
#   - Set DEBUG env or the `debug` ROS parameter to True for verbose stage
#     logging at every critical path: engine load, ROI build, first frame,
#     inference timing, pick/place transitions, service handlers. Every
#     try-block prints the exception with file/line so you can grep the
#     terminal for the failure point.
#   - Heartbeat thread prints a stage summary every 5s (frames/sec, last
#     inference ms, queue state, current target) so you can tell at a
#     glance whether the camera is feeding, YOLO is firing, and the loop
#     is locked onto a target.
#
# See INSTALL.md and QUICK_SETUP_HIWONDER.md for setup. See RESEARCH.md
# for the design principles.

import os
import cv2
import sys
import json
import yaml
import time
import math
import copy
import queue
import traceback
import threading
import numpy as np
from pathlib import Path

# Module-level debug toggle. ROS param `debug` can override at runtime via
# `_debug_enabled` flipping the module flag.
_DEBUG = os.environ.get('JETARM_V5_DEBUG', '0') == '1'


def _stage(tag, msg, exc=None):
    """Print a stage marker to stderr in a single grep-friendly format.
    Always prints when called (this is the primary diagnostic surface that
    the user asked for - terminal-visible problem reports at each stage)."""
    line = f'[v4][{tag}] {msg}'
    print(line, file=sys.stderr, flush=True)
    if exc is not None:
        print(f'[v4][{tag}] EXCEPTION: {type(exc).__name__}: {exc}',
              file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)


def _dbg(tag, msg):
    """Verbose stage trace - only fires when debug mode is on. Use this for
    high-volume logging (per-frame, per-tick); use `_stage` for one-shot
    pivots like 'engine loaded' or 'first frame received'."""
    if _DEBUG:
        print(f'[v4][{tag}] {msg}', file=sys.stderr, flush=True)


def _set_debug(value):
    global _DEBUG
    _DEBUG = bool(value)

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from sdk import common, fps
from app.common import Heart
from interfaces.srv import SetStringBool
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller.bus_servo_control import set_servo_position
from kinematics.kinematics_control import set_pose_target
from app.utils import calculate_grasp_yaw, position_change_detect, distortion_inverse_map

from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd

from ultralytics import YOLO

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


GRIPPER_ID = 10
PROFILES_DIR = Path(os.environ.get('JETARM_V5_PROFILES',
                                   str(Path.home() / 'jetarm_v5_profiles')))
DEFAULT_PROFILE_PATH = PROFILES_DIR / 'default.yaml'
# v5: the model-config YAML is written by the tuner UI on SAVE and read on
# startup. Kept separate from the full profile so the user can tweak the
# YOLO knobs without dragging every motion tunable along for the ride.
YOLO_CONFIG_PATH = PROFILES_DIR / 'yolo.yaml'


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
        self._paused = threading.Event()   # round 7: external pause (UI button)
        self.model = None
        self._load_count = 0
        # Runtime YOLO knobs. Updated by the v4 node from ROS params via
        # set_yolo_knobs(); used per-inference in run(). Defaults match
        # ultralytics' own defaults so behavior is identical until tuned.
        self.yolo_conf = 0.25
        self.yolo_iou = 0.7
        self.yolo_max_det = 100
        # v5: list of YOLO class IDs to KEEP. Empty/None means all classes.
        # Filtered in-engine via the predict() classes= argument
        # (works with .engine the same as .pt - filtering is post-NMS).
        self.yolo_classes = None
        # NB: no yolo_imgsz - TensorRT engines have a fixed input shape
        # baked in at compile time, so imgsz is not a runtime knob.
        self.inference_max_hz = 0.0   # 0 = uncapped
        self._last_run_t = 0.0

    def set_yolo_knobs(self, conf=None, iou=None, max_det=None,
                       hz=None, classes=None):
        if conf is not None:    self.yolo_conf = float(conf)
        if iou is not None:     self.yolo_iou = float(iou)
        if max_det is not None: self.yolo_max_det = int(max_det)
        if hz is not None:      self.inference_max_hz = float(hz)
        if classes is not None:
            # Empty list / None -> no filter (None passes through Ultralytics
            # as "all classes"). Otherwise coerce to list[int].
            if not classes:
                self.yolo_classes = None
            else:
                self.yolo_classes = [int(c) for c in classes]

    def class_names(self):
        """Return {id: name} from the loaded model (empty dict if not loaded)."""
        if self.model is None:
            return {}
        try:
            return dict(self.model.names)
        except Exception:
            return {}

    def pause(self):
        self._paused.set()

    def resume(self):
        self._paused.clear()
        self._frame_event.set()  # nudge run() out of any wait

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
            _stage('engine-swap', 'rejected - empty path')
            return False
        if not Path(path).exists():
            _stage('engine-swap', f'rejected - file missing: {path}')
            self._logger.warn(f'engine swap rejected - file missing: {path}')
            return False
        with self._swap_lock:
            self._pending_engine_path = path
        _stage('engine-swap', f'queued -> {path}')
        self._logger.info(f'engine swap queued -> {path}')
        return True

    def stop(self):
        self._stop.set()
        self._frame_event.set()

    # -- thread body --

    def _load(self, path):
        _stage('engine-load', f'starting load: {path}')
        if not Path(path).exists():
            _stage('engine-load', f'FATAL - file does not exist: {path}')
            raise FileNotFoundError(path)
        # Drop the old model first so its CUDA memory is released before the
        # new engine deserializes - critical on the 8GB Orin Nano.
        if self.model is not None:
            _stage('engine-load', 'releasing previous model')
            try:
                del self.model
            except Exception as e:
                _stage('engine-load', 'old model release raised', exc=e)
            self.model = None
            if HAS_TORCH:
                try:
                    torch.cuda.empty_cache()
                    _stage('engine-load', 'torch.cuda.empty_cache() done')
                except Exception as e:
                    _stage('engine-load', 'torch.cuda.empty_cache() failed', exc=e)
        self._logger.info(f'loading YOLO engine: {path}')
        t0 = time.time()
        m = YOLO(path, task='detect')
        _stage('engine-load', f'YOLO() constructed in {time.time()-t0:.2f}s')
        # Warmup: first inference includes engine deserialization; do it on
        # a dummy frame so the first real frame is fast.
        try:
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            tw = time.time()
            m(dummy, verbose=False)
            _stage('engine-load', f'warmup pass done in {time.time()-tw:.2f}s')
        except Exception as e:
            _stage('engine-load', 'warmup pass FAILED (continuing)', exc=e)
            self._logger.warn(f'warmup pass failed (continuing): {e}')
        self.model = m
        self._engine_path = path
        self._load_count += 1
        _stage('engine-load', f'engine active: {path} '
                              f'(total load {time.time()-t0:.2f}s, swap #{self._load_count})')
        try:
            self._on_swap(path)
        except Exception as e:
            _stage('engine-load', 'on_swap callback raised', exc=e)

    def run(self):
        _stage('inference', 'worker thread starting')
        try:
            self._load(self._engine_path)
        except Exception as e:
            _stage('inference', 'INITIAL ENGINE LOAD FAILED - worker exiting', exc=e)
            self._logger.error(f'initial engine load failed: {e}')
            return
        frames_done = 0
        while not self._stop.is_set():
            self._frame_event.wait(timeout=0.1)
            self._frame_event.clear()
            # External pause (UI 'Pause AI' button). Sleep cheaply so the
            # CPU/GPU are idle until the user resumes.
            if self._paused.is_set():
                time.sleep(0.05)
                continue
            # Service a queued hot-swap before pulling the next frame.
            with self._swap_lock:
                pending = self._pending_engine_path
                self._pending_engine_path = None
            if pending and pending != self._engine_path:
                try:
                    self._load(pending)
                except Exception as e:
                    _stage('engine-swap', f'swap to {pending} FAILED', exc=e)
                    self._logger.error(f'engine swap to {pending} failed: {e}')
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is None or self.model is None:
                continue
            # Inference rate throttle. 0 = uncapped (default).
            if self.inference_max_hz > 0.0:
                min_dt = 1.0 / self.inference_max_hz
                elapsed = time.time() - self._last_run_t
                if elapsed < min_dt:
                    time.sleep(min_dt - elapsed)
            t0 = time.time()
            self._last_run_t = t0
            try:
                # NOTE: do NOT pass imgsz=... to the predict call. TensorRT
                # engines have a fixed input shape baked in at compile time
                # (best_scaff3.engine is built at 320x320). If we pass a
                # different imgsz, Ultralytics tries to resize to that and
                # the engine assertion fails:
                #   input size (1,3,640,640) not equal to max model size (1,3,320,320)
                # Letting Ultralytics omit imgsz means it uses the engine's
                # native input shape - which is what the user trained for.
                # conf/iou/max_det are post-process NMS knobs, safe to tune.
                # classes= filters detections to a subset of class IDs
                # post-NMS; works identically for .pt and .engine. None =
                # no filter (all classes).
                results = self.model(
                    frame, verbose=False,
                    conf=self.yolo_conf, iou=self.yolo_iou,
                    max_det=self.yolo_max_det,
                    classes=self.yolo_classes)
            except Exception as e:
                _stage('inference', 'predict() raised - skipping frame', exc=e)
                self._logger.warn(f'inference error: {e}')
                continue
            with self._frame_lock:
                self._latest_result = (frame, results, t0)
            frames_done += 1
            if frames_done == 1:
                _stage('inference', f'first inference complete '
                                    f'({1000*(time.time()-t0):.0f} ms)')
            elif _DEBUG and frames_done % 30 == 0:
                # Include the YOLO knobs as actually USED on this call.
                # If the user moves the conf slider and we see conf=NEW
                # within ~30 frames, the param push path is confirmed.
                _dbg('inference', f'frames={frames_done} '
                                  f'last={1000*(time.time()-t0):.1f}ms '
                                  f'conf={self.yolo_conf:.2f} '
                                  f'iou={self.yolo_iou:.2f} '
                                  f'max_det={self.yolo_max_det}')
        _stage('inference', 'worker thread stopped')


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

    def goto_pose(self, position, pitch, duration, parallel_base=True,
                  pitch_range=(-180.0, 180.0)):
        # pitch_range: reference pick_and_place.py constrains approach/grab
        # IK to [-90, 90] so the solver picks the same elbow/wrist family
        # the vendor tuned for, and only opens to [-180, 180] for retreats.
        if self._abort:
            return None
        if self.kinematics_client is None:
            # Init couldn't find the IK service; can't move. Returning None
            # makes the pick loop treat it as a failed step and bail.
            return None
        msg = set_pose_target(position, pitch, list(pitch_range), 1.0, duration=duration)
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

    def compliance_grasp(self, target_pulse, open_pulse,
                         step=15, dwell=0.10, stall_thresh=8, timeout=2.0,
                         max_temp=65):
        """Close the gripper in small increments toward target_pulse,
        stopping the moment the jaws stall on the object (contact-stop).
        BETA / opt-in: only called when compliance_grasp_enabled is True.

        FORCE MODEL: the driver exposes no servo load/current (only
        position / voltage / temperature), so this is NOT a force sensor.
        The force limit is the SLOW APPROACH itself - each step advances
        the command by `step` pulses over `dwell` seconds, so the jaws
        inch into the object and cannot slam. Contact is detected as a
        position stall (we commanded forward, the servo didn't follow),
        and `target_pulse` (the per-class max-hold-strength cap) bounds
        how hard it can ever squeeze.

        Parameters
        ----------
        target_pulse : int   MAX hold strength for this item - never
                             squeeze past it (per-class cap).
        open_pulse   : int   Where the gripper currently is (fallback if
                             the first readback fails).
        step         : int   Pulse increment per step (smaller = gentler).
        dwell        : float Wait per step for servo motion + readback;
                             >= one 50Hz driver cycle so readback is fresh.
        stall_thresh : int   Advance under this per step = contact.
        timeout      : float Overall budget; failsafe.
        max_temp     : int   Gripper-servo over-temp cutoff (servo safety).

        Returns 'gripped'   (debounced stall on object - normal success),
                'closed'    (reached cap without contact - empty/thin jaws),
                'overheat'  (servo at/over max_temp - stopped to protect it),
                'no_feedback' (readback failed; commanded a single close),
                'aborted'.

        Robustness vs serial-bus latency / readback jitter:
        - The stall is judged on COMMANDED-vs-ACTUAL progress within the
          step (commanded_delta vs moved), not on the post-move position,
          so an undershooting earlier step can't suppress the check.
        - A 2-step debounce (`stall_streak`) rejects single-step false
          positives from position jitter / following-error.
        """
        if self._abort:
            return 'aborted'
        cur = open_pulse
        st = self.get_servo_state(GRIPPER_ID, fields=('position', 'temperature'))
        if st.get('position') is not None:
            cur = int(st['position'])
        target = int(target_pulse)
        deadline = time.time() + float(timeout)
        no_fb = 0
        stall_streak = 0
        while cur < target and time.time() < deadline:
            if self._abort:
                return 'aborted'
            prev = cur
            cmd = min(target, prev + int(step))
            commanded_delta = cmd - prev
            set_servo_position(self.joints_pub, float(dwell),
                               ((GRIPPER_ID, cmd),))
            self._sleep(dwell)
            st = self.get_servo_state(GRIPPER_ID, fields=('position', 'temperature'))
            # Servo protection: stop leaning on the gripper if it's hot.
            temp = st.get('temperature')
            if temp is not None and int(temp) >= int(max_temp):
                set_servo_position(self.joints_pub, float(dwell),
                                   ((GRIPPER_ID, prev),))
                _stage('grip', f'over-temp {int(temp)}C >= {int(max_temp)}C - stopping')
                return 'overheat'
            actual = st.get('position')
            if actual is None:
                no_fb += 1
                # No readback: fall back to one blocking close so the pick
                # can still finish, then bail (can't do contact detection
                # blind).
                if no_fb >= 3:
                    self.set_gripper(target, max(0.2, dwell))
                    return 'no_feedback'
                continue
            actual = int(actual)
            moved = actual - prev
            cur = actual
            # Contact = we asked the jaws to advance meaningfully but they
            # didn't. Judged on this step's commanded-vs-actual, immune to
            # earlier undershoot.
            if commanded_delta > int(stall_thresh) and moved < int(stall_thresh):
                stall_streak += 1
                if stall_streak >= 2:
                    # Hold at the current position (fresh same-value command
                    # so the servo's holding torque engages).
                    set_servo_position(self.joints_pub, float(dwell),
                                       ((GRIPPER_ID, cur),))
                    return 'gripped'
            else:
                stall_streak = 0
        return 'closed'


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

class ObjectSortingNodeV5(Node):

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
        # ---- Model config (BUFFERED in tuner UI - applied on SAVE only) ----
        ('engine_path', '/home/ubuntu/third_party_ros2/data/best_scaff2.engine', None),
        ('yolo_conf_thresh', 0.25, (0.05, 0.95)),
        ('yolo_iou_thresh',  0.7,  (0.10, 0.90)),
        ('yolo_max_det',     100,  (1, 300)),
        # JSON list of YOLO class IDs to KEEP after NMS. Empty list = all
        # classes. Populated automatically from model.names after the
        # first engine load and surfaced as per-class checkboxes in the
        # tuner UI's Model tab.
        ('yolo_enabled_classes', '[]', None),
        # ---- Detection lock/movement gating ----
        ('lock_distance_thresh', 0.005, (0.001, 0.05)),
        ('count_still_threshold', 4, (1, 30)),
        ('count_move_threshold', 8, (1, 30)),
        ('detection_avg_frames', 3, (1, 10)),
        # ---- Motion ----
        ('motion_speed', 1.5, (0.3, 2.5)),
        ('aggression', 1.3, (0.3, 2.0)),
        ('hover_height', 0.06, (0.02, 0.15)),
        ('approach_dwell', 0.1, (0.0, 1.0)),
        ('parallel_base_motion', True, None),
        # ---- Gripping (compliance grasp - see _compliance_grasp) ----
        ('gripper_open_pulse', 200, (50, 500)),
        # Hard stop target. The compliance loop stops EARLY here if the
        # jaws stall on an object (which is the normal case for a
        # successful grip) - this is the "fully closed, nothing in jaws"
        # fallback.
        ('gripper_close_pulse', 540, (300, 700)),
        ('gripper_close_duration', 0.35, (0.1, 2.0)),
        ('gripper_settle', 0.5, (0.0, 1.5)),
        ('grab_depth', 0.02, (0.0, 0.05)),
        # ---- Force-limited grasp (BETA, opt-in) ----
        # Default OFF: the standard close-to-pulse grasp runs. When the
        # operator flips compliance_grasp_enabled (UI button), the close
        # step becomes a position-stall "close until contact" loop.
        #
        # Why position-stall? The driver's BusServoState exposes only
        # position / voltage / temperature - NO load/current field - so a
        # true force loop isn't possible without patching driver+sdk+msg.
        # Position-stall is the zero-driver-change proxy: close in small
        # increments; when the commanded pulse advances but the actual
        # position doesn't, the jaws have stalled on the object and we
        # stop. The small step + dwell IS the force limit (the jaws inch,
        # they can't slam). This is an honest contact-stop, not a sensor.
        ('compliance_grasp_enabled', False, None),  # BETA toggle
        # Per-class MAX HOLD STRENGTH: JSON {class_name: max_close_pulse}.
        # The compliance loop never squeezes past this pulse for that
        # class, so fragile cubes get a gentle cap and scaff gets a firm
        # one. Missing class => global gripper_close_pulse. Editable in
        # the tuner UI per-class targets table.
        ('grasp_strength', '{}', None),
        ('grasp_step_pulse', 15, (5, 80)),    # commanded pulse increment per step (smaller = gentler)
        ('grasp_step_dwell', 0.10, (0.03, 0.3)),  # servo motion + readback window per step (>=1 driver cycle @50Hz)
        ('grasp_stall_pulse', 8, (1, 40)),    # advance under this per step = stalled (contact)
        ('grasp_timeout', 2.0, (0.3, 5.0)),   # overall budget; failsafe
        ('grasp_max_temp', 65, (40, 80)),     # servo over-temp cutoff (protect the gripper servo)
        # ---- Misc ----
        ('startup_self_calibrate', False, None),  # v5 has no color self-cal
        ('inference_warmup', True, None),
        ('hot_log_inference_ms', False, None),
        ('debug', False, None),
        # Free-form per-target overrides as JSON-ish string, parsed lazily.
        # YOLO class names are the source of truth. Example:
        # '{"scaff": {"grab_depth": 0.025, "gripper_settle": 0.8},
        #   "cube_red": {"motion_speed": 1.8}}'
        ('target_overrides', '{}', None),
        # JSON map of YOLO class name -> [x, y, z] world coords. Where each
        # detected object gets PLACED on the map. Editable per-class in the
        # tuner UI's Places tab. Empty / class missing => fall back to the
        # hardcoded DEFAULT_PLACE_POSITIONS for that label; if there's no
        # default either, _do_place aborts (no random drops).
        # Example: '{"scaff": [-0.076, 0.16, 0.015], "cube_red": [0.064, 0.23, 0.015]}'
        ('place_positions', '{}', None),
        # Frame-rate throttles. 0 = uncapped.
        ('inference_max_hz', 0.0,  (0.0, 60.0)),
        ('publish_max_hz',   0.0,  (0.0, 60.0)),
        ('publish_scale',    1.0,  (0.25, 1.0)),
        ('publish_jpeg_quality', 80, (30, 95)),
        # Independent stop/start (UI buttons map to these).
        ('enable_camera_sub', True, None),
        ('enable_inference',  True, None),
    )

    def __init__(self, name='custom_sortingv5'):
        _stage('init', f'constructing node {name!r}')
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Apply default profile (if present) BEFORE declaring tunables, so its
        # values become the seed values rather than getting overwritten.
        try:
            self._seeded_from_default = self._apply_default_profile_seed()
        except Exception as e:
            _stage('init', 'default profile seed failed (continuing)', exc=e)
            self._seeded_from_default = {}
        try:
            self._declare_tunables()
        except Exception as e:
            _stage('init', 'declare_tunables FAILED', exc=e)
            raise
        self.add_on_set_parameters_callback(self._on_param_change)

        # Honour `debug` param if the user declared it via overrides.
        try:
            if self.has_parameter('debug') and bool(self.get_parameter('debug').value):
                _set_debug(True)
                _stage('init', 'debug mode ON (ROS param)')
        except Exception:
            pass
        if _DEBUG:
            _stage('init', f'environment: CAMERA_TYPE={os.environ.get("CAMERA_TYPE")} '
                           f'CHASSIS_TYPE={os.environ.get("CHASSIS_TYPE")} '
                           f'need_compile={os.environ.get("need_compile")}')

        # ---- Shared state ----
        # v5: no HED/OpenCV color preprocessing. Single YOLO model is the
        # whole detection pipeline; class names come from model.names at
        # engine-load time and are mirrored into target_labels.
        self.bridge = CvBridge()

        self.lock = threading.RLock()
        self.fps = fps.FPS()
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        if 'CAMERA_TYPE' not in os.environ:
            _stage('init', 'CAMERA_TYPE env var is missing - downstream code WILL fail. '
                           'Did the launcher source the Hiwonder env?')
        self.camera_type = os.environ.get('CAMERA_TYPE', 'GEMINI')

        self.tag_size = 0.025

        self.place_position = copy.deepcopy(self.DEFAULT_PLACE_POSITIONS)
        self.place_offsets = {k: [0.0, 0.0, 0.0] for k in self.place_position}

        # target_labels is REPLACED at engine-load time with one entry per
        # class in the model. Initial seed = the place_position keys so the
        # node has something sensible to filter on if YOLO hasn't loaded yet.
        self.target_labels = {k: True for k in self.place_position}
        self.detection_history = {}
        self.running = True
        self._init_state()

        # ---- Callback groups (research recommendation #1) ----
        self.cam_group = MutuallyExclusiveCallbackGroup()      # camera + sorting loop
        self.svc_group = ReentrantCallbackGroup()              # services / clients

        # ---- Pubs / subs ----
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        # Default reliability (RELIABLE), depth 10. ros2 humble's
        # image_view and web_video_server both subscribe with RELIABLE
        # by default - matching them is what makes them deliver frames.
        # depth=10 (was 1) gives the slower consumers room to drain
        # without the pub queue overwriting at high fps.
        self.result_publisher = self.create_publisher(
            Image, '/custom_sortingv5/image_result', 10)
        # JPEG sibling for remote/browser viewing. The <base>/compressed
        # naming follows the image_transport convention so rqt_image_view
        # lists it as a transport and web_video_server's type=ros_compressed
        # streams the pre-encoded frames without re-encoding server-side
        # (raw Image here is ~23 MB/s; JPEG q80 is ~1-2 MB/s). Frames are
        # only encoded when someone is subscribed (see _publish_image).
        self.compressed_publisher = self.create_publisher(
            CompressedImage, '/custom_sortingv5/image_result/compressed', 10)
        # Machine-readable heartbeat mirror for the tuner UI: same data as
        # the 5s log heartbeat, as a JSON String the UI subscribes to.
        self.status_publisher = self.create_publisher(String, '~/status', 1)

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
        # v5: model-config save service. The tuner UI buffers
        # engine_path / yolo_conf / yolo_iou / yolo_max_det / classes
        # in the Model tab and only calls this on SAVE. data_str is the
        # JSON-encoded config; data_bool=true means "persist to
        # yolo.yaml in the profiles dir" (false = apply but don't write).
        self.create_service(SetStringBool, '~/save_yolo_config',
                            self.save_yolo_config_srv_callback,
                            callback_group=self.svc_group)

        # ---- Service clients ----
        # NB: kinematics comes up via the SDK launch (which our launch
        # includes). It takes a few seconds after process start to appear.
        # If it never shows we DON'T raise - we keep the node alive so the
        # camera-always-on subscription still feeds the live viewer and
        # the operator can see what's happening. Pick/place will be gated
        # at runtime by the `self.kinematics_client is None` check.
        _stage('init', 'waiting for kinematics/set_pose_target service...')
        self.kinematics_client = self.create_client(SetRobotPose,
                                                    'kinematics/set_pose_target',
                                                    callback_group=self.svc_group)
        if not self.kinematics_client.wait_for_service(timeout_sec=60.0):
            _stage('init', 'kinematics/set_pose_target NEVER appeared in 60s - '
                           'continuing without it. Live camera view will still '
                           'work; pick/place will fail when attempted. Check: '
                           'sudo journalctl -u start_app_node.service -n 80 --no-pager')
            self.kinematics_client = None
        else:
            _stage('init', 'kinematics/set_pose_target ready')
        self.set_joint_value_target_client = self.create_client(
            SetJointValue, 'kinematics/set_joint_value_target',
            callback_group=self.svc_group)
        if not self.set_joint_value_target_client.wait_for_service(timeout_sec=10.0):
            _stage('init', 'kinematics/set_joint_value_target not seen in 10s - continuing')
        self.bus_servo_state_client = self.create_client(
            GetBusServoState, 'ros_robot_controller/bus_servo/get_state',
            callback_group=self.svc_group)
        if not self.bus_servo_state_client.wait_for_service(timeout_sec=5.0):
            _stage('init', 'bus_servo/get_state unavailable - vision-only fallback')
            self.get_logger().warn('bus_servo/get_state unavailable - vision-only fallback')
            self.bus_servo_state_client = None
        else:
            _stage('init', 'bus_servo/get_state ready (full feedback enabled)')

        self.motion = MotionController(self, self.joints_pub,
                                       self.kinematics_client,
                                       self.bus_servo_state_client)

        # ---- Inference worker (one CUDA context, hot-swappable) ----
        try:
            self.inference = InferenceWorker(self.p('engine_path'), self.get_logger(),
                                             on_swap=self._on_engine_loaded)
            # Push runtime YOLO knobs from ROS params into the worker.
            self.inference.set_yolo_knobs(
                conf=self.p('yolo_conf_thresh'),
                iou=self.p('yolo_iou_thresh'),
                max_det=self.p('yolo_max_det'),
                hz=self.p('inference_max_hz'))
            self.inference.start()
            # Honor enable_inference at startup (if user has it preset to false).
            if not bool(self.p('enable_inference')):
                self.inference.pause()
            _stage('init', f'inference worker started (engine={self.p("engine_path")})')
        except Exception as e:
            _stage('init', 'inference worker FAILED to start', exc=e)
            raise

        # ---- Camera subscriptions: BEST_EFFORT, depth=1 --------------------
        # ALWAYS-ON from process startup so the operator can see the camera
        # feed the moment v4 boots - the camera should be "ready to run even
        # when the robot is stopped". Round 15: image_callback also submits
        # every frame to the InferenceWorker regardless of enable_sorting so
        # YOLO knob tuning is visible live while stopped. The explicit
        # enable_inference param / Pause AI button is the GPU kill switch.
        self.intrinsic = None
        self.distortion = None
        self._latest_raw_bgr = None
        # Round 14: overlay-data architecture. sorting_loop stores DRAWING
        # PRIMITIVES (bboxes, contours, labels, lock-line) into _latest_overlay
        # instead of a full annotated image. The 30Hz republisher reads the
        # latest raw frame from _latest_raw_bgr and composites the overlay on
        # top before publishing. Live background stays at camera rate even
        # when detection lags.
        self._latest_overlay = None
        self.image_sub = self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback,
            1, callback_group=self.cam_group)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback,
            1, callback_group=self.cam_group)
        _stage('init', 'camera subscriptions created (always-on) - '
                       'topic /depth_cam/rgb/image_raw + camera_info')

        self._startup_done = False
        self._frames_received = 0
        self._first_frame_logged = False
        self._first_camera_info_logged = False
        self._last_hb_frames = 0
        self._last_hb_time = time.time()
        self.create_timer(0.0, self._startup, callback_group=self.svc_group)
        # Periodic heartbeat so the operator can see at a glance whether
        # the pipeline is alive (camera fed, YOLO firing, target locked).
        self.create_timer(5.0, self._heartbeat, callback_group=self.svc_group)
        # Low-rate raw-frame republisher: keeps ~/image_result alive (so the
        # rqt_image_view / browser viewer always shows the camera) even when
        # sorting is off. When sorting IS on, sorting_loop publishes its own
        # annotated frame and this tick becomes a no-op.
        # 33 ms = 30 Hz publish tick (was 15 Hz). The viewer/web_video_server
        # can paint at this rate; publishing faster than the viewer paints is
        # wasted work, but 30 Hz gives a smooth view without overcommitting.
        # publish_max_hz still throttles inside _publish_image.
        self.create_timer(0.033, self._raw_republish_tick,
                          callback_group=self.svc_group)

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
        # so the seed actually wins over hardcoded defaults. One at a time:
        # a single type-mismatched value must not abort the whole seed.
        if self._seeded_from_default:
            applied = 0
            for k, v in self._seeded_from_default.items():
                try:
                    self.set_parameters([rclpy.parameter.Parameter(k, value=v)])
                    applied += 1
                except Exception as e:
                    _stage('init', f'default profile param {k}={v!r} rejected', exc=e)
            self.get_logger().info(
                f'loaded default profile from {DEFAULT_PROFILE_PATH} '
                f'({applied}/{len(self._seeded_from_default)} params)')

    def _apply_yolo_knob(self, name, value):
        kw = {}
        if name == 'yolo_conf_thresh': kw['conf'] = value
        elif name == 'yolo_iou_thresh': kw['iou'] = value
        elif name == 'yolo_max_det':   kw['max_det'] = value
        elif name == 'inference_max_hz': kw['hz'] = value
        if kw:
            self.inference.set_yolo_knobs(**kw)

    def _flush_pending_yolo_knobs(self):
        if not (hasattr(self, 'inference') and self.inference is not None):
            return
        # The ROS params were already updated when the user moved the slider
        # - only the application to the InferenceWorker was deferred. Re-sync
        # ALL knobs from the current param values instead of replaying the
        # queue: the queue can be dropped (enter's _init_state recreates it)
        # while the params always hold the latest values, so an unconditional
        # sync on every sorting-off transition is self-healing.
        if self._pending_yolo_knobs:
            queued = ', '.join(f'{k}={v}' for k, v in self._pending_yolo_knobs.items())
            _stage('param', f'flushing queued YOLO knobs: {queued}')
            self._pending_yolo_knobs.clear()
        self.inference.set_yolo_knobs(
            conf=self.p('yolo_conf_thresh'),
            iou=self.p('yolo_iou_thresh'),
            max_det=self.p('yolo_max_det'),
            hz=self.p('inference_max_hz'))

    def _on_param_change(self, params):
        for p in params:
            # If the engine path was changed via param, queue a hot-swap.
            if p.name == 'engine_path' and isinstance(p.value, str) and p.value:
                if hasattr(self, 'inference') and self.inference is not None:
                    self.inference.request_engine_swap(p.value)
            if p.name == 'debug':
                _set_debug(bool(p.value))
                _stage('param', f'debug mode -> {bool(p.value)}')
            # Live-update YOLO knobs as the user moves the sliders, but only
            # while sorting is OFF — changing confidence mid-pick would change
            # what the lock loop sees and could destabilise an in-flight grab.
            # When sorting is ON we queue the value and flush it the moment
            # the user toggles sorting off.
            # yolo_imgsz is intentionally excluded - it's a build-time
            # property of the TRT engine, see TUNABLE_PARAMS comment.
            if p.name in ('yolo_conf_thresh', 'yolo_iou_thresh',
                          'yolo_max_det', 'inference_max_hz') \
                    and hasattr(self, 'inference') and self.inference is not None:
                if self.enable_sorting:
                    self._pending_yolo_knobs[p.name] = p.value
                    _stage('param', f'{p.name} -> {p.value} QUEUED '
                                    f'(sorting on, applies when off)')
                else:
                    self._apply_yolo_knob(p.name, p.value)
                    _stage('param', f'{p.name} -> {p.value} APPLIED')
            # Independent stop/start of the orbbec subscription.
            if p.name == 'enable_camera_sub':
                want = bool(p.value)
                if want and self.image_sub is None:
                    self.image_sub = self.create_subscription(
                        Image, '/depth_cam/rgb/image_raw',
                        self.image_callback, 1, callback_group=self.cam_group)
                    _stage('param', 'enable_camera_sub -> True (subscription RECREATED)')
                elif not want and self.image_sub is not None:
                    try:
                        self.destroy_subscription(self.image_sub)
                    except Exception as e:
                        _stage('param', 'failed to destroy image_sub', exc=e)
                    self.image_sub = None
                    _stage('param', 'enable_camera_sub -> False (subscription DROPPED)')
            # Pause / resume the inference worker.
            if p.name == 'enable_inference':
                if hasattr(self, 'inference') and self.inference is not None:
                    if bool(p.value):
                        self.inference.resume()
                        _stage('param', 'enable_inference -> True (worker RESUMED)')
                    else:
                        self.inference.pause()
                        _stage('param', 'enable_inference -> False (worker PAUSED)')
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
        # YOLO knob changes received while sorting is ON are stashed here
        # and flushed when sorting toggles back OFF.
        self._pending_yolo_knobs = {}
        # Last calibrate result for the heartbeat mirror.
        self._last_calibrate = None
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

    def _heartbeat(self):
        """Periodic stage summary - prints once every ~5s. Tells you at a
        glance whether the pipeline is healthy. Always-on (cheap)."""
        now = time.time()
        dt = max(0.001, now - self._last_hb_time)
        fps = (self._frames_received - self._last_hb_frames) / dt
        self._last_hb_frames = self._frames_received
        self._last_hb_time = now
        latest = self.inference.latest() if hasattr(self, 'inference') else None
        last_inf_ms = -1.0
        if latest is not None:
            try:
                last_inf_ms = 1000.0 * (time.time() - latest[2])
            except Exception:
                pass
        # Subscriber count on /custom_sortingv5/image_result. Confirms
        # that viewers are actually connected. If result_subs=0 while a
        # viewer window is open, the viewer is in a different ROS_DOMAIN
        # or its subscription dropped.
        try:
            result_subs = self.result_publisher.get_subscription_count()
        except Exception:
            result_subs = -1
        prev_subs = getattr(self, '_last_result_subs', None)
        if prev_subs is not None and result_subs != prev_subs:
            _stage('publish', f'result_subs changed: {prev_subs} -> {result_subs}')
        self._last_result_subs = result_subs
        # pub_fps: how often we're actually emitting annotated frames.
        # Distinct from cam_fps (which counts incoming orbbec frames).
        pub_count = getattr(self, '_pub_count', 0)
        last_pub_count = getattr(self, '_last_hb_pub_count', 0)
        pub_fps = max(0, (pub_count - last_pub_count)) / dt
        self._last_hb_pub_count = pub_count
        # Camera + AI on/off state for at-a-glance UI mirror.
        cam_sub_state = 'LIVE' if self.image_sub is not None else 'PAUSED'
        ai_state = 'PAUSED' if (hasattr(self, 'inference') and self.inference
                                and self.inference._paused.is_set()) else 'LIVE'
        sorting_alive = not hasattr(self, '_sorting_thread') or self._sorting_thread.is_alive()
        _stage('heartbeat',
               f'enter={self.enter} sorting={self.enable_sorting} '
               f'cam={cam_sub_state} ai={ai_state} '
               f'cam_fps={fps:.1f} pub_fps={pub_fps:.1f} '
               f'frames={self._frames_received} result_subs={result_subs} '
               f'roi={"ok" if len(self.roi) else "NONE"} '
               f'intrinsic={"ok" if self.intrinsic is not None else "NONE"} '
               f'inference_age_ms={last_inf_ms:.0f} '
               f'target={self.target[0] if self.target else "none"} '
               f'transport={self.start_transport} '
               f'sorting_thread={"alive" if sorting_alive else "DEAD"}')
        if not sorting_alive:
            _stage('heartbeat', 'CRITICAL: sorting_loop thread is DEAD — see CRASHED log above')
        # Publish the same data as JSON for the tuner UI's live mirror.
        try:
            engine = ''
            class_names = {}
            if hasattr(self, 'inference') and self.inference is not None:
                engine = os.path.basename(self.inference._engine_path or '')
                class_names = self.inference.class_names()
            self.status_publisher.publish(String(data=json.dumps({
                'ts': now,
                'enter': bool(self.enter),
                'sorting': bool(self.enable_sorting),
                'cam': cam_sub_state,
                'ai': ai_state,
                'cam_fps': round(fps, 1),
                'pub_fps': round(pub_fps, 1),
                'frames': int(self._frames_received),
                'result_subs': int(result_subs),
                'roi': bool(len(self.roi)),
                'inference_age_ms': round(last_inf_ms, 0),
                'target': self.target[0] if self.target else '',
                'transport': bool(self.start_transport),
                'sorting_thread_alive': bool(sorting_alive),
                'engine': engine,
                # v5: class names from model.names (id -> name). The tuner
                # UI auto-populates the Classes filter + Places tab when
                # this changes.
                'class_names': {str(k): v for k, v in class_names.items()},
                'last_calibrate': self._last_calibrate,
            })))
        except Exception as e:
            if not getattr(self, '_status_pub_warned', False):
                _stage('heartbeat', 'status publish failed (one-time warn)', exc=e)
                self._status_pub_warned = True

    def _startup(self):
        if self._startup_done:
            return
        self._startup_done = True
        _stage('startup', 'spawning sorting + transport threads')
        self._sorting_thread = threading.Thread(
            target=self._sorting_loop_wrapped, daemon=True, name='sorting-loop')
        self._sorting_thread.start()
        threading.Thread(target=self._transport_thread_wrapped, daemon=True, name='transport').start()
        # Build the ROI at boot (not just on enter) so detection overlays
        # appear in the viewer immediately - the launch default is
        # start:=false, and before this the user saw no bounding boxes
        # until the first START press, defeating tune-while-stopped.
        self.start_get_roi = True
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
        # v5: startup_self_calibrate is a no-op (kept as a tunable for
        # backward profile compat). The LAB color self-cal is gone.
        self.create_service(Trigger, '~/init_finish',
                            lambda req, resp: setattr(resp, 'success', True) or resp,
                            callback_group=self.svc_group)
        _stage('startup', 'v4 init finish')
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
        cfg_file = self.config_path + self.config_file
        _stage('roi', f'reading transform.yaml from {cfg_file}')
        try:
            with open(cfg_file, 'r') as f:
                config = yaml.safe_load(f)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
            _stage('roi', f'  yaml loaded: extristric.shape={extristric.shape} '
                          f'corners.shape={corners.shape} '
                          f'white_area_center.shape={self.white_area_center.shape}')
        except FileNotFoundError as e:
            _stage('roi', f'transform.yaml NOT FOUND - have you run calibration?', exc=e)
            raise
        except KeyError as e:
            _stage('roi', f'transform.yaml missing required key {e!s} - '
                          f'rerun calibration_node.launch.py', exc=e)
            raise
        except Exception as e:
            _stage('roi', 'transform.yaml load failed', exc=e)
            raise
        waited = 0.0
        while waited < 30.0:
            if self.intrinsic is not None and self.distortion is not None:
                break
            time.sleep(0.05); waited += 0.05
        else:
            # PREVIOUSLY this was a silent `return` which left self.roi == []
            # and let the caller clear start_get_roi - so the loop never
            # retried. Raise instead so the caller's except fires and we
            # keep trying until intrinsics arrive.
            _stage('roi', 'gave up waiting for camera intrinsics after 30s '
                          '(camera_info topic never arrived)')
            raise TimeoutError('camera_info never arrived')
        try:
            intr_shape = self.intrinsic.shape if hasattr(self.intrinsic, 'shape') else type(self.intrinsic)
            dist_shape = self.distortion.shape if hasattr(self.distortion, 'shape') else type(self.distortion)
            _stage('roi', f'  intrinsic ready after {waited:.2f}s '
                          f'(shape={intr_shape}) distortion.shape={dist_shape}')
        except Exception:
            pass
        try:
            tvec = extristric[:1]
            rmat = extristric[1:]
            tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)),
                                                       np.array(rmat), 0.03)
            self.extristric = tvec, rmat
            _stage('roi', '  extrinsic plane shift done')
        except Exception as e:
            _stage('roi', 'extristric_plane_shift FAILED', exc=e)
            raise
        try:
            imgpts, _ = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec),
                                          self.intrinsic, self.distortion)
            imgpts = np.int32(imgpts).reshape(-1, 2)
            _stage('roi', f'  projected {len(imgpts)} corners to image pixels')
        except Exception as e:
            _stage('roi', 'cv2.projectPoints FAILED - check self.intrinsic shape '
                          f'(currently {getattr(self.intrinsic, "shape", "?")}, '
                          'cv2 wants 3x3)', exc=e)
            raise
        if len(imgpts) == 0:
            _stage('roi', 'projectPoints returned zero pixels - cannot build ROI')
            raise RuntimeError('empty imgpts')
        x_min = min(imgpts, key=lambda p: p[0])[0]
        x_max = max(imgpts, key=lambda p: p[0])[0]
        y_min = min(imgpts, key=lambda p: p[1])[1]
        y_max = max(imgpts, key=lambda p: p[1])[1]
        self.roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
        _stage('roi', f'  computed roi (y={self.roi[0]}..{self.roi[1]} '
                      f'x={self.roi[2]}..{self.roi[3]}) on intrinsic '
                      f'shape={getattr(self.intrinsic, "shape", "?")}')

    # ------------------------------------------------------------------ services

    def enter_srv_callback(self, request, response):
        # Camera subs are created in __init__ now (always-on), so enter no
        # longer touches them. It still does the v2/v4-compat work of:
        #   * resetting transient state (counters, last_position, ...)
        #   * starting the heartbeat watchdog
        #   * requesting an ROI rebuild
        #   * moving the arm home
        _stage('enter', 'enter requested (camera subs were already alive)')
        self.get_logger().info('enter v4')
        self._init_state()
        try:
            self.heart = Heart(self, '~/heartbeat', 5,
                               lambda _: self.exit_srv_callback(Trigger.Request(),
                                                                Trigger.Response()))
        except Exception as e:
            _stage('enter', 'Heart() failed to construct - continuing without it', exc=e)
            self.heart = None
        self.start_get_roi = True
        joint_angle = [500, 520, 210, 50, 500]
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, joint_angle[1]),
                                                (3, joint_angle[2]), (4, joint_angle[3]),
                                                (5, 500), (10, self.p('gripper_open_pulse'))))
        self.enter = True
        response.success = True
        return response

    def exit_srv_callback(self, request, response):
        # Camera subs stay alive across enter/exit so the live viewer keeps
        # working when the robot is stopped. exit just halts sorting/motion.
        if self.enter:
            if self.heart is not None:
                self.heart.destroy(); self.heart = None
            self.enter = False
            self.start_transport = False
            self.motion.abort(True)
        # exit is a stop path too (heartbeat watchdog calls it): clear the
        # sorting flag so the lock branch can't re-trigger transport, and
        # land any YOLO knob changes that were queued while sorting was on.
        self.enable_sorting = False
        self._flush_pending_yolo_knobs()
        response.success = True
        return response

    def enable_sorting_srv_callback(self, request, response):
        _stage('svc', f'enable_sorting -> {bool(request.data)}')
        self.motion.abort(not request.data)
        self.enable_sorting = bool(request.data)
        if self.enable_sorting:
            # Fresh start: drop any stale lock so a target seen before the
            # stop can't fire start_transport on the very next frame with
            # pre-stop count_still already over the threshold.
            self.target = None
            self.count_still = 0
            self.count_move = 0
            self.last_position = None
            self.target_miss_count = 0
            self.detection_history = {}
        else:
            # When sorting toggles OFF, apply any YOLO knob changes that
            # were queued while sorting was ON so the user's tweaks land.
            self._flush_pending_yolo_knobs()
        response.success = True
        return response

    def set_target_srv_callback(self, request, response):
        _stage('svc', f'set_target {request.data_str}={request.data_bool}')
        if request.data_str in self.target_labels:
            self.target_labels[request.data_str] = request.data_bool
        else:
            _stage('svc', f'  unknown label "{request.data_str}" - ignored')
        response.success = True
        return response

    def recalibrate_srv_callback(self, request, response):
        # v5 calibrate: verifies the things that actually matter for
        # accurate picking, with NO color self-cal (gone with the LAB
        # pipeline). The button checks: camera intrinsics arrived,
        # transform.yaml reloaded + ROI rebuilt, IK service alive, and
        # IK actually solves at each configured place position. Optionally
        # tours the arm over those positions so the operator can eyeball
        # accuracy. State is reset too so any stale lock is cleared.
        _stage('svc', 'calibrate requested')
        threading.Thread(target=self._calibrate, daemon=True).start()
        response.success = True
        return response

    def _calibrate(self):
        _stage('calibrate', 'starting')
        self._init_state()
        self.start_get_roi = True

        # 1. Wait briefly for intrinsics + ROI rebuild
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if (self.intrinsic is not None and self.distortion is not None
                    and len(self.roi) > 0 and self.white_area_center is not None):
                break
            time.sleep(0.2)
        ok_intr = self.intrinsic is not None and self.distortion is not None
        ok_roi = len(self.roi) > 0
        _stage('calibrate', f'intrinsics={ok_intr} roi={ok_roi}')

        # 2. Verify IK service ready
        ok_ik = (self.kinematics_client is not None
                 and self.kinematics_client.wait_for_service(timeout_sec=2.0))
        _stage('calibrate', f'kinematics_client ready={ok_ik}')

        # 3. Try to IK-solve each configured place position WITHOUT moving.
        # If IK fails for a position, the user has it outside the arm's
        # reachable workspace - the place would silently bail at runtime
        # otherwise.
        place_results = {}
        try:
            user_pp = json.loads(self.p('place_positions') or '{}')
        except Exception:
            user_pp = {}
        candidates = dict(self.place_position)
        if isinstance(user_pp, dict):
            candidates.update(user_pp)
        for label, pos in candidates.items():
            if not isinstance(pos, (list, tuple)) or len(pos) != 3:
                place_results[label] = 'malformed'
                continue
            try:
                msg = set_pose_target([float(pos[0]), float(pos[1]),
                                        float(pos[2]) + 0.05],
                                       80, [-90.0, 90.0], 1.0, duration=1.0)
                if self.kinematics_client is None:
                    place_results[label] = 'no_ik_service'
                    continue
                fut = self.kinematics_client.call_async(msg)
                # Spin the future on this thread (the executor will service
                # it via the svc_group ReentrantCallbackGroup).
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 1.5:
                    time.sleep(0.02)
                res = fut.result() if fut.done() else None
                if res is None or not res.pulse:
                    place_results[label] = 'unreachable'
                else:
                    place_results[label] = 'ok'
            except Exception as e:
                place_results[label] = f'err:{type(e).__name__}'
        for k, v in place_results.items():
            _stage('calibrate', f'  place[{k}] -> {v}')

        # 4. Optional physical tour: drive the arm above each reachable
        # place position briefly so the operator can EYEBALL accuracy.
        # Triggered by the second-stage tour flag in the request - the
        # default is just verify (no motion). Honor sorting=OFF as a
        # safety gate too.
        tour = (self.p('calibrate_tour') if 'calibrate_tour' in
                {n for n, *_ in self.TUNABLE_PARAMS} else False)
        if tour and not self.enable_sorting:
            _stage('calibrate', 'tour: driving arm over each reachable place')
            try:
                self.go_home(False)
                for label, status in place_results.items():
                    if status != 'ok':
                        continue
                    pos = candidates[label]
                    above = [float(pos[0]), float(pos[1]), float(pos[2]) + 0.08]
                    self.motion.goto_pose(above, 80, duration=1.0,
                                          parallel_base=True,
                                          pitch_range=(-90.0, 90.0))
                    self.motion._sleep(0.6)
                self.go_home(False)
            except Exception as e:
                _stage('calibrate', 'tour aborted', exc=e)

        # 5. Stash the result on the heartbeat so the UI can show pass/fail.
        self._last_calibrate = {
            'ts': time.time(),
            'intrinsics': ok_intr,
            'roi': ok_roi,
            'kinematics': ok_ik,
            'places': place_results,
        }
        ok_all = ok_intr and ok_roi and ok_ik and all(
            v == 'ok' for v in place_results.values())
        _stage('calibrate', f'done ok_all={ok_all}')

    def _on_engine_loaded(self, path):
        """InferenceWorker callback: fires from the worker thread after a
        successful engine load (initial or hot-swap). Mirrors model.names
        into target_labels so the tuner UI can render per-class checkboxes,
        and applies the YOLO classes filter from the saved config (or all
        classes if none saved)."""
        self.get_logger().info(f'engine active: {path}')
        try:
            names = self.inference.class_names()
            if names:
                # Keep an existing user toggle if the class name is still
                # present in the new model; default new classes to enabled.
                old = dict(self.target_labels)
                self.target_labels = {n: old.get(n, True) for n in names.values()}
                _stage('engine-load', f'class names from model.names: '
                                      f'{list(names.values())}')
            # Apply persisted classes filter (empty list = all classes).
            try:
                enabled_json = self.p('yolo_enabled_classes') or '[]'
                enabled = json.loads(enabled_json)
                if isinstance(enabled, list):
                    self.inference.set_yolo_knobs(classes=enabled)
            except Exception as e:
                _stage('engine-load', 'yolo_enabled_classes parse failed', exc=e)
        except Exception as e:
            _stage('engine-load', 'class-name mirror failed', exc=e)

    def save_yolo_config_srv_callback(self, request, response):
        """Apply (and optionally persist) a buffered YOLO config from the
        tuner UI's Model tab. The UI buffers slider edits locally and only
        calls this on SAVE - so changing conf/iou/classes never disturbs
        an in-flight pick, and the saved config is auto-loaded next boot.
        """
        _stage('svc', f'save_yolo_config (persist={request.data_bool})')
        try:
            cfg = json.loads(request.data_str or '{}')
        except Exception as e:
            _stage('svc', 'save_yolo_config: malformed JSON', exc=e)
            response.success = False
            return response

        # 1. Push every accepted field to the ROS param store so the
        #    rest of the system stays in sync with the saved config.
        accepted = ('engine_path', 'yolo_conf_thresh', 'yolo_iou_thresh',
                    'yolo_max_det', 'inference_max_hz')
        applied = []
        for k in accepted:
            if k not in cfg:
                continue
            try:
                self.set_parameters([rclpy.parameter.Parameter(k, value=cfg[k])])
                applied.append(k)
            except Exception as e:
                _stage('svc', f'  {k}={cfg[k]!r} rejected', exc=e)

        # 2. Classes filter: cfg['classes'] is a list[int] of YOLO class IDs.
        if 'classes' in cfg:
            try:
                classes = list(cfg['classes']) if cfg['classes'] is not None else []
                self.set_parameters([rclpy.parameter.Parameter(
                    'yolo_enabled_classes', value=json.dumps(classes))])
                # Apply to the running worker; _on_param_change ALSO does
                # this but we want it now, regardless of sorting state, so
                # the SAVE takes effect immediately.
                self.inference.set_yolo_knobs(classes=classes)
                applied.append('classes')
            except Exception as e:
                _stage('svc', f'  classes={cfg.get("classes")!r} rejected', exc=e)

        # 3. Apply conf/iou/max_det to the running worker immediately too -
        #    the user pressed SAVE, that's their explicit "land it now"
        #    signal regardless of whether sorting is on. (Normal slider
        #    moves still queue while sorting is on.)
        try:
            self.inference.set_yolo_knobs(
                conf=self.p('yolo_conf_thresh'),
                iou=self.p('yolo_iou_thresh'),
                max_det=self.p('yolo_max_det'),
                hz=self.p('inference_max_hz'))
        except Exception as e:
            _stage('svc', 'set_yolo_knobs post-save failed', exc=e)

        # 4. Engine hot-swap if engine_path changed (request_engine_swap
        #    no-ops if the path matches the currently loaded engine).
        if 'engine_path' in cfg:
            try:
                self.inference.request_engine_swap(cfg['engine_path'])
            except Exception as e:
                _stage('svc', 'engine hot-swap from save failed', exc=e)

        # 5. Persist if requested.
        if request.data_bool:
            try:
                _ensure_profiles_dir()
                payload = {k: self.p(k) for k in ('engine_path', 'yolo_conf_thresh',
                                                   'yolo_iou_thresh', 'yolo_max_det',
                                                   'inference_max_hz',
                                                   'yolo_enabled_classes')}
                with open(YOLO_CONFIG_PATH, 'w') as f:
                    yaml.safe_dump(payload, f, sort_keys=True)
                _stage('svc', f'wrote {YOLO_CONFIG_PATH}')
            except Exception as e:
                _stage('svc', f'write {YOLO_CONFIG_PATH} failed', exc=e)
                response.success = False
                return response

        _stage('svc', f'save_yolo_config applied {applied}')
        response.success = True
        return response

    def load_engine_srv_callback(self, request, response):
        _stage('svc', f'load_engine -> {request.data_str}')
        ok = self.inference.request_engine_swap(request.data_str)
        if ok:
            try:
                self.set_parameters([rclpy.parameter.Parameter(
                    'engine_path', value=request.data_str)])
            except Exception as e:
                _stage('svc', 'load_engine: setting engine_path param failed', exc=e)
        response.success = ok
        return response

    def save_profile_srv_callback(self, request, response):
        _stage('svc', f'save_profile -> {request.data_str}')
        try:
            name = (request.data_str or 'profile').strip()
            if not name.endswith('.yaml'):
                name += '.yaml'
            path = PROFILES_DIR / name
            save_profile_yaml(path, self._all_tunables_dict())
            _stage('svc', f'  wrote {path}')
            self.get_logger().info(f'profile saved -> {path}')
            response.success = True
        except Exception as e:
            _stage('svc', 'save_profile FAILED', exc=e)
            self.get_logger().error(f'save_profile failed: {e}')
            response.success = False
        return response

    def load_profile_srv_callback(self, request, response):
        _stage('svc', f'load_profile -> {request.data_str}')
        try:
            name = (request.data_str or 'profile').strip()
            if not name.endswith('.yaml'):
                name += '.yaml'
            path = PROFILES_DIR / name
            params = load_profile_yaml(path)
            if not params:
                _stage('svc', f'  no params parsed from {path} (file missing or empty?)')
                response.success = False
                return response
            # Apply one at a time: a single type mismatch (YAML int where a
            # double param is declared) raises in set_parameters and would
            # abort the whole batch, silently dropping every later param.
            applied = 0
            for k, v in params.items():
                try:
                    self.set_parameters([rclpy.parameter.Parameter(k, value=v)])
                    applied += 1
                except Exception as e:
                    _stage('svc', f'  skipping param {k}={v!r}', exc=e)
            _stage('svc', f'  applied {applied}/{len(params)} params from {path}')
            self.get_logger().info(f'profile loaded <- {path} ({applied} params)')
            response.success = True
        except Exception as e:
            _stage('svc', 'load_profile FAILED', exc=e)
            self.get_logger().error(f'load_profile failed: {e}')
            response.success = False
        return response

    def save_as_default_srv_callback(self, request, response):
        _stage('svc', f'save_as_default -> {DEFAULT_PROFILE_PATH}')
        try:
            save_profile_yaml(DEFAULT_PROFILE_PATH, self._all_tunables_dict())
            self.get_logger().info(f'default profile saved -> {DEFAULT_PROFILE_PATH}')
            response.success = True
        except Exception as e:
            _stage('svc', 'save_as_default FAILED', exc=e)
            self.get_logger().error(f'save_as_default failed: {e}')
            response.success = False
        return response

    # v5: _self_calibrate was a LAB color check against the place bins,
    # removed with the rest of the OpenCV color pipeline. v5 trusts the
    # YOLO model and the static transform.yaml extrinsic.

    def _pixel_to_world(self, pixel, height=0.03):
        return self.get_object_world_position(pixel, self.intrinsic, self.extristric,
                                              self.white_area_center, height)

    # ------------------------------------------------------------------ vision

    def _detections_from_results(self, bgr_image, roi, yolo_results):
        """Run detection (YOLO + LAB color blobs) on the current frame and
        return target_info plus a primitives dict for the overlay renderer.

        Returns (target_info, primitives) where primitives is:
            {'yolo_ops': [(kind, *args), ...],  # 'rect' or 'circle'
             'color_corners': [np.intp array of 4 (x,y), ...]}
        No cv2 drawing happens in this method — the republisher applies the
        primitives to a fresh raw frame at 30Hz (see _draw_overlay).
        """
        target_info = []
        yolo_ops = []
        roi_img = bgr_image[roi[0]:roi[1], roi[2]:roi[3]]
        # v5: ALL detection comes from YOLO. The model's class names
        # (model.names) are the source of truth - no hardcoded
        # red/green/blue/scaff list, no LAB color thresholding, no HED.
        # Enabled classes are filtered by target_labels (populated from
        # model.names at engine-load time).
        for result in yolo_results:
            names = getattr(result, 'names', None) or {}
            if hasattr(result, 'obb') and result.obb is not None:
                for obb in result.obb:
                    cx, cy, w, h, r = obb.xywhr[0].cpu().numpy()
                    cls_id = int(obb.cls.cpu().numpy()) if hasattr(obb, 'cls') else 0
                    label = names.get(cls_id, f'cls_{cls_id}')
                    cx += roi[2]; cy += roi[0]
                    angle = int(math.degrees(r))
                    target_info.append([label, 1, (int(cx), int(cy)),
                                        (int(w), int(h)), angle])
                    yolo_ops.append(('circle', (int(cx), int(cy)), label))
            elif hasattr(result, 'boxes') and result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls.cpu().numpy()) if hasattr(box, 'cls') else 0
                    label = names.get(cls_id, f'cls_{cls_id}')
                    cx = (x1 + x2) / 2 + roi[2]
                    cy = (y1 + y2) / 2 + roi[0]
                    w, h = x2 - x1, y2 - y1
                    target_info.append([label, 1, (int(cx), int(cy)),
                                        (int(w), int(h)), 0])
                    yolo_ops.append(('rect',
                                     (int(x1 + roi[2]), int(y1 + roi[0])),
                                     (int(x2 + roi[2]), int(y2 + roi[0])),
                                     label))
        primitives = {'yolo_ops': yolo_ops, 'color_corners': []}
        return target_info, primitives

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

    def _per_target_overrides(self, label):
        try:
            import json
            raw = self.p('target_overrides') or '{}'
            d = json.loads(raw)
            return d.get(label, {})
        except Exception:
            return {}

    def _do_pick(self, position, pitch, yaw, label):
        # v5 single-shot pick (mirrors example/app/object_sorting.py:355).
        # No retry loop. The close step uses compliance_grasp() to stop the
        # moment the jaws stall on the object - that's the "grip until
        # closed on the item" behavior. If the whole pick fails, the
        # transport thread opens the gripper, goes home, and the next
        # detection iteration re-locks naturally.
        ov = self._per_target_overrides(label)
        speed_p = float(ov.get('motion_speed', self.p('motion_speed')))
        speed = max(0.1, 1.0 / speed_p)
        aggression = float(ov.get('aggression', self.p('aggression')))
        hover_h = float(self.p('hover_height'))
        approach_dwell = float(self.p('approach_dwell'))
        open_pulse = int(self.p('gripper_open_pulse'))
        close_pulse = int(ov.get('gripper_close_pulse', self.p('gripper_close_pulse')))
        close_dur = float(ov.get('gripper_close_duration', self.p('gripper_close_duration'))) * speed
        settle = float(ov.get('gripper_settle', self.p('gripper_settle')))
        grab_depth = float(ov.get('grab_depth', self.p('grab_depth')))
        parallel_base = bool(self.p('parallel_base_motion'))
        use_compliance = bool(self.p('compliance_grasp_enabled'))
        # Per-class MAX HOLD STRENGTH (compliance only): never squeeze past
        # this for this class. Falls back to the global close pulse.
        max_strength = int(self._grasp_strength_for(label, close_pulse))
        # Compliance knobs (per-target override capable).
        grasp_step = int(ov.get('grasp_step_pulse', self.p('grasp_step_pulse')))
        grasp_dwell = float(ov.get('grasp_step_dwell', self.p('grasp_step_dwell')))
        grasp_stall = int(ov.get('grasp_stall_pulse', self.p('grasp_stall_pulse')))
        grasp_timeout = float(ov.get('grasp_timeout', self.p('grasp_timeout')))
        grasp_max_temp = int(self.p('grasp_max_temp'))

        if self.motion.aborted:
            return False
        _dbg('pick', f'pick {label} compliance={use_compliance} '
                     f'close={close_pulse} max_strength={max_strength} '
                     f'grab_depth={grab_depth:.3f} settle={settle:.2f}')
        hover = [position[0], position[1], position[2] + hover_h]
        if self.motion.goto_pose(hover, pitch,
                                 duration=max(0.5, 1.1 * speed / aggression),
                                 parallel_base=parallel_base,
                                 pitch_range=(-90.0, 90.0)) is None:
            _stage('pick', 'hover IK failed')
            self.motion.set_gripper(open_pulse, 0.2 * speed)
            return False
        self.motion.set_wrist(yaw, 0.5 * speed)
        self.motion.set_gripper(open_pulse, 0.2 * speed)
        self.motion._sleep(approach_dwell)
        descend = [position[0], position[1], position[2] - grab_depth]
        if self.motion.goto_pose(descend, pitch,
                                 duration=max(0.35, 0.7 * speed / aggression),
                                 parallel_base=False,
                                 pitch_range=(-90.0, 90.0)) is None:
            _stage('pick', 'descend IK failed')
            self.motion.set_gripper(open_pulse, 0.2 * speed)
            return False
        # Close: STANDARD fixed-pulse grasp by default; opt-in BETA
        # compliance "close until contact" with a per-class strength cap.
        if use_compliance:
            outcome = self.motion.compliance_grasp(
                target_pulse=max_strength, open_pulse=open_pulse,
                step=grasp_step, dwell=grasp_dwell,
                stall_thresh=grasp_stall, timeout=grasp_timeout,
                max_temp=grasp_max_temp)
            _stage('pick', f'compliance_grasp -> {outcome}')
        else:
            self.motion.set_gripper(close_pulse, close_dur)
        # Settle so the servo holding torque has time to bite before lift.
        self.motion._sleep(settle)
        if self.motion.goto_pose(hover, pitch,
                                 duration=max(0.35, 0.7 * speed / aggression),
                                 parallel_base=False) is None:
            _stage('pick', 'lift IK failed')
            return False
        _stage('pick', f'pick {label} complete ({outcome})')
        return True

    def _grasp_strength_for(self, label, default_pulse):
        """Per-class MAX HOLD STRENGTH cap (max close pulse) for the
        compliance grasp. Reads the grasp_strength JSON {class: pulse};
        falls back to the global close pulse when the class isn't set."""
        try:
            strengths = json.loads(self.p('grasp_strength') or '{}')
            if isinstance(strengths, dict) and label in strengths:
                return int(strengths[label])
        except Exception as e:
            _stage('grip', 'grasp_strength parse failed', exc=e)
        return int(default_pulse)

    def _resolve_place_position(self, label):
        """Look up the place coords for a YOLO class label. UI-editable
        place_positions JSON dict wins; falls back to DEFAULT_PLACE_POSITIONS
        for legacy labels (red/green/blue/scaff); returns None if neither
        knows about the label so _do_place can refuse to drop somewhere
        random."""
        try:
            user = json.loads(self.p('place_positions') or '{}')
            if isinstance(user, dict) and label in user:
                pos = user[label]
                if isinstance(pos, (list, tuple)) and len(pos) == 3:
                    return [float(pos[0]), float(pos[1]), float(pos[2])]
                _stage('place', f'place_positions[{label}] malformed: {pos!r}')
        except Exception as e:
            _stage('place', 'place_positions parse failed', exc=e)
        return copy.deepcopy(self.place_position.get(label))

    def _do_place(self, label):
        speed = max(0.1, 1.0 / float(self.p('motion_speed')))
        aggression = float(self.p('aggression'))
        position = self._resolve_place_position(label)
        if position is None:
            _stage('place', f'no place position configured for {label!r} '
                            f'- set one in the tuner UI Places tab')
            return False
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
        # Let the jaws fully open before retreating so the object isn't
        # dragged (reference sleeps 0.5s after release).
        self.motion._sleep(float(self.p('gripper_settle')))
        if self.motion.goto_pose(hover, 80,
                                 duration=max(0.35, 0.6 * speed / aggression),
                                 parallel_base=False) is None:
            return False
        return True

    def _sorting_loop_wrapped(self):
        try:
            self.sorting_loop()
        except Exception as e:
            _stage('sorting-loop', 'CRASHED - thread exiting', exc=e)

    def _transport_thread_wrapped(self):
        try:
            self.transport_thread()
        except Exception as e:
            _stage('transport', 'CRASHED - thread exiting', exc=e)

    def transport_thread(self):
        while self.running:
            if not self.start_transport:
                time.sleep(0.05); continue
            try:
                position, yaw, target = self.transport_info
                if position[0] > 0.22:
                    position[2] += 0.01
                position = self._apply_kinematics_calibration(position)
                label = target[0]
                _stage('transport', f'BEGIN pick {label} at '
                                    f'({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) '
                                    f'yaw_pulse={yaw}')
                t0 = time.time()
                picked = self._do_pick(position, 80, yaw, label)
                _stage('transport',
                       f'pick {"OK" if picked else "FAIL"} in {time.time()-t0:.2f}s')
                if picked:
                    tp = time.time()
                    placed = self._do_place(label)
                    _stage('transport',
                           f'place {"OK" if placed else "FAIL"} in {time.time()-tp:.2f}s')
                    self.go_home(False)
                    if not placed:
                        self.get_logger().warn(f'place failed for {label}')
                else:
                    self.motion.set_gripper(int(self.p('gripper_open_pulse')), 0.3)
                    self.go_home(True)
            except Exception as e:
                _stage('transport', 'transport cycle CRASHED - returning home', exc=e)
                try:
                    self.go_home(True)
                except Exception:
                    pass
            self.target = None
            self.start_transport = False

    # ------------------------------------------------------------------ sorting loop

    def sorting_loop(self):
        _stage('sorting-loop', 'thread started, waiting for enter+frames')
        avg_frames = 1
        ticks = 0
        while self.running:
            # Detection runs whenever frames are arriving - no enter gate -
            # so YOLO knob changes are visible in the viewer with sorting
            # OFF, including on a fresh boot before the first START. The
            # pick trigger (start_transport) is gated on self.enable_sorting
            # further down, and exit/STOP clears enable_sorting, so no
            # motion can start from this loop while stopped.
            latest = self.inference.latest()
            if latest is None:
                time.sleep(0.005); continue
            ticks += 1
            if ticks == 1:
                _stage('sorting-loop', 'first inference result reached the loop')
            bgr_image, results, ts = latest
            # Round 14: viewer is fed by _raw_republish_tick which always
            # grabs the latest raw frame and composites the overlay below.
            # No passthrough stash needed here.
            if self.start_get_roi and self.intrinsic is not None and self.distortion is not None:
                try:
                    self.get_roi()
                except Exception as e:
                    _stage('sorting-loop', 'get_roi raised - retrying next tick', exc=e)
                    time.sleep(0.5); continue
                # Belt-and-braces: even if get_roi() somehow returns without
                # raising AND without setting self.roi, keep retrying instead
                # of latching start_get_roi=False (which was the v4 silent
                # failure that left roi=NONE forever).
                if not (hasattr(self.roi, '__len__') and len(self.roi)):
                    _stage('sorting-loop',
                           'get_roi returned but self.roi is empty - retrying')
                    time.sleep(0.5); continue
                self.start_get_roi = False
                _stage('sorting-loop', 'ROI built - detection branch is now live')
            try:
                roi = self.roi.copy() if len(self.roi) else []
                intrinsic = self.intrinsic
                avg_frames = max(1, int(self.p('detection_avg_frames')))
                still_thresh = int(self.p('count_still_threshold'))
                move_thresh = int(self.p('count_move_threshold'))
                lock_thresh = float(self.p('lock_distance_thresh'))
                lock_line = None
                primitives = {'yolo_ops': [], 'color_corners': []}
                if len(roi) > 0 and not self.start_transport and intrinsic is not None:
                    target_info, primitives = self._detections_from_results(bgr_image, roi, results)
                    if target_info and self.last_object_info_list:
                        target_info = position_change_detect.position_reorder(
                            target_info, self.last_object_info_list, 20)
                    self.last_object_info_list = copy.deepcopy(target_info)
                    # Lock + transport trigger only when sorting is enabled.
                    # When OFF the detection above still fires (so overlays
                    # appear in the viewer for YOLO tuning) but no pick runs.
                    if self.enable_sorting:
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
                                    lock_line = (result[1][0], result[1][1])
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
                                    _stage('sorting-loop',
                                           f'LOCK -> {t[0]} at '
                                           f'({avg_pos[0]:.3f},{avg_pos[1]:.3f},{avg_pos[2]:.3f}) '
                                           f'yaw_pulse={yaw_pulse} '
                                           f'(avg over {len(hist)} frame{"" if len(hist)==1 else "s"})')
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
                    target_info = []
                # Stash the overlay dict for _raw_republish_tick to composite
                # onto the latest raw frame at 30Hz. The viewer is updated by
                # the timer, not from here.
                self._latest_overlay = {
                    'ts': time.time(),
                    'targets': copy.deepcopy(target_info),
                    'yolo_ops': primitives['yolo_ops'],
                    'color_corners': primitives['color_corners'],
                    'lock_line': lock_line,
                    'inf_ms': 1000.0 * (time.time() - ts) if self.p('hot_log_inference_ms') else None,
                }
                _dbg('sorting-loop', f'iteration done loop_lag_ms={1000*(time.time()-ts):.0f} '
                                     f'targets={len(target_info)}')
                time.sleep(0.001)
            except Exception as e:
                _stage('sorting-loop', 'UNHANDLED exception — loop continuing', exc=e)
                time.sleep(0.1)

    def _publish_image(self, bgr):
        try:
            # Publish-rate throttle (0 = uncapped). Useful when the viewer
            # paints slower than we can publish - dropping pub frames
            # saves CPU on cv_bridge + DDS without hurting the viewer.
            now = time.time()
            pub_hz = float(self.p('publish_max_hz'))
            if pub_hz > 0.0:
                min_dt = 1.0 / pub_hz
                if now - getattr(self, '_last_pub_t', 0.0) < min_dt:
                    return
            self._last_pub_t = now
            # Optional downsample before publish. publish_scale=0.5 ->
            # 320x240, ~4x less bytes to ship and paint. Uses INTER_AREA
            # which is the right pick for downscale.
            scale = float(self.p('publish_scale'))
            if scale > 0.0 and scale < 1.0:
                h, w = bgr.shape[:2]
                bgr = cv2.resize(bgr, (max(1, int(w * scale)), max(1, int(h * scale))),
                                 interpolation=cv2.INTER_AREA)
            # Some upstream paths can hand us a numpy view (sliced ROI,
            # transposed, etc.) which causes cv_bridge to emit a message
            # with a stride that doesn't match `step = width * channels`.
            if not bgr.flags['C_CONTIGUOUS']:
                bgr = np.ascontiguousarray(bgr)
            msg = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            self.result_publisher.publish(msg)
            # JPEG sibling for remote viewers (~23 MB/s raw vs ~1-2 MB/s
            # JPEG q80). Encode ONLY while someone is subscribed -
            # imencode costs ~2-4ms/frame on the Orin CPU.
            try:
                if self.compressed_publisher.get_subscription_count() > 0:
                    quality = int(self.p('publish_jpeg_quality'))
                    ok, enc = cv2.imencode(
                        '.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
                    if ok:
                        cmsg = CompressedImage()
                        cmsg.header = msg.header
                        cmsg.format = 'jpeg'
                        cmsg.data = enc.tobytes()
                        self.compressed_publisher.publish(cmsg)
            except Exception as e:
                if not getattr(self, '_compressed_warned', False):
                    _stage('publish', 'compressed publish failed (one-time warn)', exc=e)
                    self._compressed_warned = True
            # Track pub rate for heartbeat.
            self._pub_count = getattr(self, '_pub_count', 0) + 1
            if not getattr(self, '_first_publish_logged', False):
                self._first_publish_logged = True
                _stage('publish', f'first frame published: shape={bgr.shape} '
                                  f'step={msg.step} '
                                  f'contig={bool(bgr.flags["C_CONTIGUOUS"])} '
                                  f'qos=reliable/depth10 '
                                  f'topic=/custom_sortingv5/image_result')
        except Exception as e:
            _stage('camera', 'cv_bridge output publish failed', exc=e)

    # ------------------------------------------------------------------ camera cbs

    def camera_info_callback(self, msg):
        try:
            self.intrinsic = np.asmatrix(np.array(msg.k, dtype=np.float64).reshape(3, 3))
            self.distortion = np.array(msg.d)
            if not self._first_camera_info_logged:
                self._first_camera_info_logged = True
                _stage('camera', f'first camera_info received: '
                                 f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                                 f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        except Exception as e:
            _stage('camera', 'camera_info parse failed', exc=e)

    def image_callback(self, ros_rgb_image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(ros_rgb_image, 'bgr8')
        except Exception as e:
            _stage('camera', 'cv_bridge frame transformation failed', exc=e)
            return

        self._frames_received += 1
        if not self._first_frame_logged:
            self._first_frame_logged = True
            _stage('camera', f'FIRST FRAME received: {bgr.shape} '
                             f'enc={ros_rgb_image.encoding}')
        if _DEBUG and self._frames_received % 30 == 0:
            _dbg('cam-recv', f'frame #{self._frames_received} shape={bgr.shape} '
                             f'enc={ros_rgb_image.encoding}')
        self._latest_raw_bgr = bgr
        # Inference runs whenever the worker is not paused (enable_inference
        # param / Pause AI button). This lets the user tune YOLO sliders with
        # sorting OFF and see detections live in the viewer — the pick
        # trigger is gated separately in sorting_loop.
        self.inference.submit(bgr)
        if _DEBUG and self._frames_received % 30 == 0:
            _dbg('cam-infer', f'frame #{self._frames_received} submitted to InferenceWorker')

    def _draw_overlay(self, bgr, overlay):
        """Apply detection overlay primitives in-place onto a fresh raw frame.
        Called from the 30Hz republisher. The overlay dict is produced once
        per sorting_loop iteration (~0.4-30 fps depending on HED) but the
        composited frame is published at full 30Hz, so the live camera
        background updates fluidly even when detection lags."""
        if overlay is None:
            return
        for op in overlay.get('yolo_ops', ()):
            kind = op[0]
            if kind == 'rect':
                cv2.rectangle(bgr, op[1], op[2], (0, 0, 255), 2)
            elif kind == 'circle':
                cv2.circle(bgr, op[1], 8, (0, 0, 255), -1)
        for corners in overlay.get('color_corners', ()):
            cv2.drawContours(bgr, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)
        for t in overlay.get('targets', ()):
            label = t[0]
            cx, cy = t[2]
            cv2.putText(bgr, label,
                        (cx - 4 * len(label + str(t[1])), cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        lock_line = overlay.get('lock_line')
        if lock_line is not None:
            cv2.line(bgr, lock_line[0], lock_line[1],
                     (255, 255, 0), 2, cv2.LINE_AA)
        inf_ms = overlay.get('inf_ms')
        if inf_ms is not None:
            cv2.putText(bgr, f'inf {inf_ms:.0f}ms', (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

    def _raw_republish_tick(self):
        """30 Hz publisher: always grabs the latest raw camera frame and
        composites the latest detection overlay onto it. The live background
        updates at camera rate even when the detection loop is slow (HED
        per-frame can drop it to ~0.4 fps); overlays may lag by 0.5-2s but
        the user can see the robot moving in real time.
        """
        bgr_src = self._latest_raw_bgr
        if bgr_src is None:
            return
        bgr = bgr_src.copy()
        # ROI-not-yet-ready hint. ROI now builds at boot (see _startup), so
        # this shows whenever the build hasn't completed, not just mid-sort.
        if not (hasattr(self.roi, '__len__') and len(self.roi)):
            cv2.putText(bgr, 'ROI: building...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        # Composite overlay whenever detection has run. Detection runs
        # regardless of enable_sorting (see sorting_loop) so the user can
        # tune YOLO knobs with sorting OFF and watch the bboxes update live.
        overlay = self._latest_overlay
        if overlay is not None:
            age = time.time() - overlay['ts']
            if age > 2.0:
                last_warn = getattr(self, '_last_stale_warn_age', -999)
                if age - last_warn >= 30.0:
                    _stage('publish', f'WARNING: overlay is {age:.1f}s stale '
                                      f'- detection loop may be stuck or slow')
                    self._last_stale_warn_age = age
            elif age < 2.0:
                self._last_stale_warn_age = -999
            # Don't paint ghost boxes over a live background: when the AI is
            # paused (or the detection loop wedges) the overlay stops
            # refreshing while the camera keeps moving. 5s tolerates the
            # worst-case HED-on iteration (~2.5s) without flicker.
            if age <= 5.0:
                try:
                    self._draw_overlay(bgr, overlay)
                except Exception as e:
                    if not getattr(self, '_overlay_err_warned', False):
                        _stage('camera', 'overlay draw failed (one-time warn)', exc=e)
                        self._overlay_err_warned = True
        try:
            self._publish_image(bgr)
        except Exception as e:
            if not getattr(self, '_pub_warned', False):
                _stage('camera', 'republish failed (one-time warn)', exc=e)
                self._pub_warned = True

def main():
    _stage('main', 'starting custom_sortingv5')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    try:
        rclpy.init()
    except Exception as e:
        _stage('main', 'rclpy.init() FAILED', exc=e); raise
    try:
        node = ObjectSortingNodeV5('custom_sortingv5')
    except Exception as e:
        _stage('main', 'Node construction FAILED - see stage prints above', exc=e)
        raise
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    _stage('main', 'spinning executor (4 threads)')
    try:
        executor.spin()
    except KeyboardInterrupt:
        _stage('main', 'KeyboardInterrupt - shutting down')
        node.running = False
        try:
            node.inference.stop()
        except Exception as e:
            _stage('main', 'inference.stop() raised', exc=e)
        executor.shutdown()
    except Exception as e:
        _stage('main', 'executor.spin() raised', exc=e)
        raise

if __name__ == '__main__':
    main()
