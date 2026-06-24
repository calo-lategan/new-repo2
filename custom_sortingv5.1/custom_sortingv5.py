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
import functools
import traceback
import threading
import numpy as np
from pathlib import Path

# Module-level debug toggle. ROS param `debug` can override at runtime via
# `_debug_enabled` flipping the module flag.
_DEBUG = os.environ.get('JETARM_V5_DEBUG', '0') == '1'

# ---- Session log file (Round 12 commit Z) ----
# Every _stage() line is mirrored to a per-session log file the user can push
# to the GitHub repo via tools/push_logs.sh. Survives process exit; rotated
# at boot to keep only the last 20 sessions. NO camera frames are written -
# text only.
_LOG_DIR = os.environ.get(
    'JETARM_V5_LOG_DIR',
    os.path.expanduser('~/jetarm_v5/logs'))
_LOG_FILE = None
_LOG_LOCK = threading.Lock()
_SESSION_TS = time.strftime('%Y-%m-%d_%H-%M-%S')

def _open_session_log():
    """Open the session log file once per process. Idempotent; failures are
    swallowed so a broken filesystem never takes down the node."""
    global _LOG_FILE
    if _LOG_FILE is not None:
        return _LOG_FILE
    try:
        os.makedirs(_LOG_DIR, exist_ok=True)
        # Rotate: keep only the 20 most recent.
        try:
            files = sorted([f for f in os.listdir(_LOG_DIR)
                            if f.startswith('v5_session_') and f.endswith('.log')])
            for old in files[:-20]:
                try: os.remove(os.path.join(_LOG_DIR, old))
                except Exception: pass
        except Exception:
            pass
        path = os.path.join(_LOG_DIR,
                            f'v5_session_{os.getpid()}_{_SESSION_TS}.log')
        _LOG_FILE = open(path, 'a', buffering=1)  # line-buffered
        _LOG_FILE.write(f'# v5 session {_SESSION_TS} pid={os.getpid()}\n')
        return _LOG_FILE
    except Exception:
        _LOG_FILE = None
        return None


def _stage(tag, msg, exc=None):
    """Print a stage marker to stderr in a single grep-friendly format.
    Always prints when called (this is the primary diagnostic surface that
    the user asked for - terminal-visible problem reports at each stage).
    Round 12: also mirrors to ~/jetarm_v5/logs/v5_session_<pid>_<ts>.log."""
    line = f'[v4][{tag}] {msg}'
    print(line, file=sys.stderr, flush=True)
    if exc is not None:
        print(f'[v4][{tag}] EXCEPTION: {type(exc).__name__}: {exc}',
              file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)
    # Mirror to session log file (best-effort).
    f = _open_session_log()
    if f is not None:
        try:
            with _LOG_LOCK:
                f.write(f'{time.strftime("%H:%M:%S")} {line}\n')
                if exc is not None:
                    f.write(f'{time.strftime("%H:%M:%S")} [v4][{tag}] '
                            f'EXCEPTION: {type(exc).__name__}: {exc}\n')
                    f.write(traceback.format_exc())
        except Exception:
            pass


def _install_excepthook():
    """Append uncaught exceptions to the session log so crashes survive to
    the repo's logs/ folder for later analysis."""
    prev = sys.excepthook

    def _hook(exc_type, exc_value, tb):
        _stage('uncaught', f'{exc_type.__name__}: {exc_value}')
        f = _open_session_log()
        if f is not None:
            try:
                with _LOG_LOCK:
                    f.write(''.join(traceback.format_exception(exc_type, exc_value, tb)))
            except Exception:
                pass
        if prev:
            prev(exc_type, exc_value, tb)

    sys.excepthook = _hook


_install_excepthook()


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
from std_msgs.msg import String, Bool
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
from kinematics.kinematics_control import set_pose_target, set_joint_value_target
from app.utils import calculate_grasp_yaw, position_change_detect, distortion_inverse_map
# Round 14 CC: vendor positioning utilities. calculate_world_position is
# THE function vendor uses to land the arm on objects - importing
# instead of re-deriving keeps v5 gripping in lockstep with vendor.
try:
    from app.utils import utils as vendor_utils
    from app.utils import search_plane as vendor_search_plane
    from app.utils import calculate_grasp_yaw_by_depth as vendor_grasp_depth
    _VENDOR_UTILS_OK = True
except Exception as _e:
    vendor_utils = None
    vendor_search_plane = None
    vendor_grasp_depth = None
    _VENDOR_UTILS_OK = False
    print(f'[v4][init] WARN: vendor utils not importable: {_e}',
          file=sys.stderr, flush=True)

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

    def __init__(self, engine_path, logger, on_swap=None, task_override='auto'):
        super().__init__(daemon=True, name='yolo-inference')
        self._logger = logger
        self._on_swap = on_swap or (lambda _: None)
        self._engine_path = engine_path
        self._task = None
        self._task_override = task_override or 'auto'
        self._pending_engine_path = None
        self._pending_force_reload = False
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

    def set_task_override(self, value):
        """Set the per-engine task hint (auto|detect|obb|segment|pose|classify).
        Caller is responsible for triggering an engine reload to pick it up."""
        self._task_override = (value or 'auto').strip().lower() or 'auto'

    def request_engine_swap(self, path, force=False):
        """Queue an engine swap. With force=True, re-init even if the path
        matches the currently loaded engine (Reload-engine UI path)."""
        if not path:
            _stage('engine-swap', 'rejected - empty path')
            return False
        if not Path(path).exists():
            _stage('engine-swap', f'rejected - file missing: {path}')
            self._logger.warn(f'engine swap rejected - file missing: {path}')
            return False
        with self._swap_lock:
            self._pending_engine_path = path
            if force:
                self._pending_force_reload = True
        verb = 'reload' if force else 'swap'
        _stage('engine-swap', f'queued {verb} -> {path}')
        self._logger.info(f'engine {verb} queued -> {path}')
        return True

    def swap_pending(self):
        """True if an engine swap/reload is queued and not yet serviced."""
        with self._swap_lock:
            return self._pending_engine_path is not None

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
        override = (self._task_override or '').strip().lower()
        if override and override != 'auto':
            m = YOLO(path, task=override)
        else:
            m = YOLO(path)
        raw = getattr(m, 'task', None)
        self._task = (raw or override or 'detect')
        _stage('engine-load', f'YOLO() constructed in {time.time()-t0:.2f}s '
                              f'task={self._task} (raw={raw!r}, '
                              f'override={override!r})')
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
                force = self._pending_force_reload
                self._pending_engine_path = None
                self._pending_force_reload = False
            if pending and (force or pending != self._engine_path):
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
            # v5.1 FIX (crash): clamp to >= 0. The `while time.time() < end`
            # check can pass with end-now a sub-microsecond positive, then time
            # advances past `end` before this line evaluates `end - time.time()`
            # -> negative -> time.sleep(negative) -> "ValueError: sleep length
            # must be non-negative", which crashed the transport cycle
            # intermittently (caught by transport_thread's except, self-recovered).
            time.sleep(max(0.0, min(0.02, end - time.time())))

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

    def get_servo_state(self, servo_id, fields=('position',), timeout=0.6):
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
        res = self._await_future(future, timeout=timeout)
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

    def get_arm_servo_positions(self):
        """v5.1 bin-teach: read arm servos 1..5 -> [p1, p2, p3, p4, p5] pulses
        (or None). Uses the PROVEN single-servo get_servo_state path five times
        in series. (An earlier one-shot bulk request with five GetBusServoCmd
        entries returned fewer than five states on this driver build, so
        joint-jog failed with 'cannot read servo positions' even though the
        service was up.)"""
        if self.bus_servo_state_client is None:
            return None
        pulses = []
        for sid in (1, 2, 3, 4, 5):
            st = self.get_servo_state(sid, fields=('position',), timeout=1.5)
            p = st.get('position') if isinstance(st, dict) else None
            if p is None:
                return None
            pulses.append(int(p))
        return pulses

    def set_servo(self, servo_id, pulse, duration):
        """v5.1 bin-teach: command ONE arm servo to an absolute pulse,
        clamped to the hard SDK range 0..1000 (the vendor IK solver clamps
        to the same range). Returns the clamped pulse actually commanded."""
        p = max(0, min(1000, int(pulse)))
        set_servo_position(self.joints_pub, float(duration),
                           ((int(servo_id), p),))
        self._sleep(duration)
        return p

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
        # Round 16 JJ: the loaded YOLO engine reports these EXACT class
        # names, so the place map must key on them or _do_place refuses
        # the drop. Each class gets a distinct zone on the drop grid.
        'scaff':           [-0.076, 0.16, 0.015],  # hazardous-waste zone
        'red cube':        [ 0.064, 0.23, 0.015],
        'green cube':      [-0.006, 0.23, 0.015],
        'light blue cube': [-0.076, 0.23, 0.015],
        'dark blue cube':  [-0.006, 0.16, 0.015],
        # Legacy keys kept for back-compat with older engines / tags.
        'green': [-0.006, 0.23, 0.015],
        'red':   [ 0.064, 0.23, 0.015],
        'blue':  [-0.076, 0.23, 0.015],
        'tag1':  [-0.076, 0.16, 0.015],
        'tag2':  [-0.006, 0.16, 0.015],
        'tag3':  [ 0.064, 0.16, 0.015],
    }

    TUNABLE_PARAMS = (
        # name, default, range_or_None
        # ---- Model config (BUFFERED in tuner UI - applied on SAVE only) ----
        ('engine_path', '/home/ubuntu/third_party_ros2/data/best_scaff2.engine', None),
        # ultralytics infers task from engine metadata, but engines built via
        # trtexec / custom export don't carry that field, so YOLO falls back
        # to 'detect' silently. Set this per engine to force the task.
        ('engine_task', 'auto', None),
        # Production sorting baseline (decently-high conf, few detections,
        # moderate NMS since items don't overlap). default.yaml ships the same.
        ('yolo_conf_thresh', 0.60, (0.05, 0.95)),
        ('yolo_iou_thresh',  0.45, (0.10, 0.90)),
        ('yolo_max_det',     5,    (1, 300)),
        # JSON list of YOLO class IDs to KEEP after NMS. Empty list = all
        # classes. Populated automatically from model.names after the
        # first engine load and surfaced as per-class checkboxes in the
        # tuner UI's Model tab.
        ('yolo_enabled_classes', '[]', None),
        # ---- Detection lock/movement gating ----
        # Round 17 NN.3: vendor-exact thresholds. The Round 16 looser values
        # were a workaround for the index-drift bug; NN.1 fixes it by
        # matching on label only, so we can return to vendor parity.
        ('lock_distance_thresh', 0.005, (0.001, 0.05)),
        ('count_still_threshold', 10, (1, 30)),
        ('count_move_threshold', 10, (1, 30)),
        # Vendor uses instantaneous positions (no multi-frame averaging).
        ('detection_avg_frames', 1, (1, 10)),
        # ---- Manual offsets (calibration-free nudge) ----
        # Pixel-space: shifts detection coords so the overlay boxes sit on
        # the objects (also corrects the pixel fed to the world projection).
        ('detection_offset_x', 0, (-300, 300)),   # +x shifts detections RIGHT (px)
        ('detection_offset_y', 0, (-300, 300)),   # +y shifts detections DOWN (px)
        # World-space: shifts the final pick position (metres) so the arm
        # lands on the object, compensating calibration without the AprilTag.
        # Range widened in Round 11: ±0.50 m covers significant tag drift.
        ('grip_offset_x', 0.0, (-0.5, 0.5)),
        ('grip_offset_y', 0.0, (-0.5, 0.5)),
        # Workspace XY scale (the user's "Z" - how big the world-map appears).
        # Multiplies the projected world XY around (0,0) AFTER grip_offset,
        # so 1.05 stretches the workspace 5% outward from centre. 1.0 = no
        # change.
        ('workspace_scale', 1.0, (0.5, 1.5)),
        # Physical workspace dimensions (m). The yellow overlay rectangle is
        # drawn at this world size around the AprilTag centre, so the user
        # can size it to match their physical mat and confirm the centre is
        # under the arm.
        ('workspace_size_x', 0.30, (0.05, 1.0)),
        ('workspace_size_y', 0.30, (0.05, 1.0)),
        # Calibration overlay state machine. UI sets this:
        #   'off'    no extra overlay (default)
        #   'manual' persistent overlay while the user nudges the sliders
        #   'flash'  one-shot overlay set by _run_vendor_calibration on
        #            success; a worker flips it back to 'off' after
        #            calibrate_flash_secs so the user sees the result briefly.
        ('calibrate_overlay_mode', 'off', None),
        ('calibrate_flash_secs', 6.0, (1.0, 30.0)),
        # ---- Grasp orientation ----
        # Vendor `calculate_gripper_yaw_angle` picks min |yaw| - fine for
        # square cubes, wrong for elongated objects (scaff) where the gripper
        # ends up wrapping the LONG axis. When True, prefer the orientation
        # that puts jaws clamping on the SHORT OBB axis (narrow waist);
        # tiebreak by min |yaw|. Only kicks in for aspect ratio
        # |w-h|/max(w,h) >= grasp_short_axis_min_ratio (squares stay vendor).
        ('grasp_prefer_short_axis', True, None),
        ('grasp_short_axis_min_ratio', 0.10, (0.02, 0.5)),
        # v5.1 (B4): hardware-tunable gripper yaw offset (degrees) added to the
        # OBB angle before computing the grasp yaw. ultralytics OBB defines `w`
        # as the LONG side and `r` (radians, normalized to [-45,135)) as the
        # angle of `w` (confirmed against the ultralytics OBB task docs), so the
        # short-axis-clamp logic is correct in software. But if the physical
        # gripper is mounted 90 deg off the wrist-servo zero, the jaws end up
        # across the WRONG axis. Set this to 90 (or -90) on the arm to correct
        # it from a profile/preset instead of editing code. 0 = no change.
        ('grasp_yaw_offset_deg', 0, (-180, 180)),
        # v5.1 (A2): when compliance grasp is on and the gripper servo can't
        # confirm contact ('no_feedback'), fail the pick instead of placing a
        # possibly empty gripper. Set False only if this arm's gripper servo
        # never reports position (else every grasp would no_feedback->fail).
        ('grasp_fail_on_no_feedback', True, None),
        # v5.1 (A4): retreat to a safe transit pose (vendor [0.11,0,0.09],
        # wrist centred) after a grab, before traversing to the drop zone.
        ('safe_transit_enabled', True, None),
        # ---- Depth integration (Round 11) ----
        # Use the depth camera to derive per-object Z height so the arm
        # descends to the actual top of each cube instead of the hardcoded
        # table-plane assumption. Falls back gracefully if the depth topic
        # isn't published or no fresh frames arrive.
        ('use_depth_for_z', True, None),
        ('depth_window_px', 5, (1, 30)),
        # Sanity gate: drop detections whose median depth is outside this
        # band (above table = floating, hand; below table = reflection).
        # Once a table plane is fit (Y2), the gate becomes "above plane"
        # and depth_min_z_m is effectively unused.
        ('depth_min_z_m', -0.02, (-0.10, 0.20)),
        ('depth_max_z_m',  0.20, (0.05, 0.50)),
        # Overlay: paint a JET-colormapped depth heatmap inside the
        # workspace ROI so the user can SEE whether depth is reading the
        # cubes (Round 12 Y5).
        ('overlay_depth_view', False, None),
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
        # v5.1 (depth-accurate grasp height): hard safety floor for the descend
        # Z. The pick descends to (object-top depth Z - grab_depth); this clamps
        # the result so a noisy/wrong depth reading can never drive the gripper
        # below this world Z into the table. Raise it if the arm ever dips too
        # low; lower it (toward the table-plane Z) only if it stops short.
        ('min_descend_z_m', -0.02, (-0.10, 0.10)),
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
        # Seconds the test-grip pose holds open BEFORE closing, so the
        # operator can place an object between the jaws.
        ('test_grip_dwell', 3.0, (0.5, 10.0)),
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
        # Frame-rate throttles. 0 = uncapped. inference capped at 15 Hz by
        # default - picks happen ~1-2 Hz so uncapped just pegs the GPU.
        ('inference_max_hz', 15.0, (0.0, 60.0)),
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
        # v5.1 FIX (C1): the vendor calibration_node chooses its config_path by
        # chassis type - calibration.py: `if self.chassis_type=='Slide_Rails':
        # config_path = .../stepper/config/ else .../app/config/`. launch_v5.sh
        # defaults CHASSIS_TYPE=Slide_Rails, so v5 reading app/config
        # unconditionally meant a fresh Position/Depth calibration was written
        # to stepper/config while v5 kept reading a stale app/config file (or
        # none). Key off the SAME env var the vendor uses so reads == writes.
        # CONFIRMED (operator): this JetArm IS a Slide_Rails arm - 'Slide_Rails'
        # is its lead-screw linear-rail axis, NOT a mobile chassis. The vendor
        # kinematics (driver/kinematics/transform.py), pick (pick_and_place.py)
        # and calibration all branch on CHASSIS_TYPE=Slide_Rails, so it is left
        # as-is and v5 correctly reads .../stepper/config/transform.yaml.
        _chassis = os.environ.get('CHASSIS_TYPE', '')
        if _chassis == 'Slide_Rails':
            self.config_path = "/home/ubuntu/ros2_ws/src/stepper/config/"
        else:
            self.config_path = "/home/ubuntu/ros2_ws/src/app/config/"
        _stage('init', f'config_path={self.config_path} '
                       f'(CHASSIS_TYPE={_chassis!r})')
        if 'CAMERA_TYPE' not in os.environ:
            _stage('init', 'CAMERA_TYPE env var is missing - downstream code WILL fail. '
                           'Did the launcher source the Hiwonder env?')
        self.camera_type = os.environ.get('CAMERA_TYPE', 'GEMINI')

        self.tag_size = 0.025

        self.place_position = copy.deepcopy(self.DEFAULT_PLACE_POSITIONS)
        # v5.1 bin-teach: the commanded teach/jog world pose [x,y,z]. None until
        # the first jog/goto (lazily seeded). Tracked here because there is no
        # get_current_pose service - we follow the pose WE command. Lives in
        # __init__ (not _init_state) so it survives ~/enter.
        self._teach_pose = None
        self._teach_running = False
        # v5.1 bin-teach (joint jog): the last-read arm servo pulses [p1..p5]
        # and the FK world readout derived from them. _teach_center/_teach_edges
        # buffer the workspace-teach captures until 'save_workspace'. _last_teach_msg
        # mirrors the most recent teach status to the heartbeat for the UI.
        self._teach_joints = None
        self._teach_center = None
        self._teach_edges = []
        self._last_teach_msg = ''
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
        # v5.1 bin-teach: a low-latency status mirror published the instant a
        # teach action finishes (the 5s heartbeat is too slow for interactive
        # jogging). Carries the live world readout, servo pulses, staged
        # workspace centre/edges and the last status message.
        self.teach_status_publisher = self.create_publisher(
            String, '~/teach_status', 1)

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
        # v5: reload the CURRENT engine in place. Tuner UI "Reload engine"
        # button. No path change - just re-init the YOLO model (releases the
        # old CUDA context, reloads the .engine, re-runs the warmup pass).
        self.create_service(Trigger, '~/reload_engine',
                            self.reload_engine_srv_callback,
                            callback_group=self.svc_group)
        # v5: model-config save service. The tuner UI buffers
        # engine_path / yolo_conf / yolo_iou / yolo_max_det / classes
        # in the Model tab and only calls this on SAVE. data_str is the
        # JSON-encoded config; data_bool=true means "persist to
        # yolo.yaml in the profiles dir" (false = apply but don't write).
        self.create_service(SetStringBool, '~/save_yolo_config',
                            self.save_yolo_config_srv_callback,
                            callback_group=self.svc_group)
        # v5: generic "apply (and optionally persist) a dict of params". Backs
        # every per-tab "Save & Apply" button in the tuner UI. data_str is a
        # JSON {param: value} map; data_bool=true merges those keys into
        # default.yaml so they survive relaunch. Applying via set_parameters
        # reuses _on_param_change (engine swap + YOLO knob apply).
        self.create_service(SetStringBool, '~/apply_and_persist',
                            self.apply_and_persist_srv_callback,
                            callback_group=self.svc_group)
        # v5 BETA: per-class grip-test orchestrator. UI Places tab calls
        # this. Drives the arm to a safe test pose, dwells so the operator
        # can place an object between the jaws, then runs compliance_grasp
        # with the per-class strength cap so the user can tune it
        # interactively without firing a full pick cycle.
        self.create_service(SetStringBool, '~/test_grip',
                            self.test_grip_srv_callback,
                            callback_group=self.svc_group)
        # v5: CALIBRATE orchestrator. The UI button calls this; the node
        # drives the vendor AprilTag calibration node (enter/start/finish/
        # exit) and rebuilds ROI from the transform.yaml it writes. Owning
        # this server-side guarantees sorting is stopped before the vendor
        # 'enter' moves the arm, and only the node can refresh the ROI.
        self.create_service(Trigger, '~/run_calibration',
                            self.run_calibration_srv_callback,
                            callback_group=self.svc_group)
        # Round 12 Y6: refit the table plane without re-running AprilTag.
        self.create_service(Trigger, '~/depth_plane_refit',
                            self._depth_plane_refit_srv,
                            callback_group=self.svc_group)
        # v5.1 bin-teach: jog the arm in world X/Y/Z, GOTO an existing class
        # bin, or SAVE the current arm pose as place_positions[<class>]. JSON
        # command in data_str; data_bool = persist to default.yaml.
        self.create_service(SetStringBool, '~/teach',
                            self.teach_srv_callback,
                            callback_group=self.svc_group)

        # Round 14 AA: LAB color threshold calibration (one-for-one with
        # vendor lab_manager.py). 7 services + a mono8 mask publisher.
        # Reuses the SAME lab_config.yaml schema vendor sorting reads.
        # Round 15: LAB color calibration is now delegated to the vendor
        # lab_manager_node (launched alongside us by the v5 launch file).
        # The Color tab in tune_uiv5 talks to /lab_manager/* directly.

        # ---- Vendor calibration node clients (idle until CALIBRATE) ----
        # Created, not blocked on: if the calibration node isn't running we
        # degrade gracefully (run_calibration logs + returns failure).
        self.calib_enter_cli = self.create_client(
            Trigger, '/calibration/enter', callback_group=self.svc_group)
        self.calib_start_cli = self.create_client(
            Trigger, '/calibration/start', callback_group=self.svc_group)
        self.calib_exit_cli = self.create_client(
            Trigger, '/calibration/exit', callback_group=self.svc_group)
        self._calib_finished = threading.Event()
        self.create_subscription(
            Bool, '/calibration/finish', self._on_calib_finish,
            1, callback_group=self.svc_group)

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
                                             on_swap=self._on_engine_loaded,
                                             task_override=self.p('engine_task'))
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
        # Depth subscription for per-object Z height. Try the aligned topic
        # first (depth_to_color is depth pre-warped to the colour intrinsics,
        # so depth[v,u] indexes by colour pixel directly). Fall back to the
        # raw depth topic if the aligned one doesn't exist. Subscribing to a
        # non-existent topic is a no-op in rclpy - we set _depth_available
        # when a frame actually arrives.
        self._depth_available = False
        self._latest_depth = None
        self._latest_depth_ts = 0.0
        self._depth_topic = ''
        # v5.1 FIX (depth alignment): only use the depth-world path when the
        # depth stream is COLOUR-ALIGNED to the RGB frame. This Orbbec only
        # publishes RAW depth (640x400) on a different FOV than the RGB
        # (640x480), and _depth_at samples depth at the RGB pixel - so on the
        # raw stream the depth value is for the WRONG physical point, which
        # corrupts the depth-derived world position (observed as X~0.30 m and
        # negative Z). When depth is not aligned we fall back to the legacy
        # AprilTag homography (correct for this fixed camera). Only the
        # /depth_cam/depth_to_color stream sets this True.
        self._depth_is_aligned = False
        self._depth_first_logged = False
        self._depth_cam_info = None
        # Round 12 Y1: depth->color TF (depth_cam_link -> depth_cam_color_frame).
        # Set asynchronously after the static TF arrives. None until then.
        self._depth_to_color_tf = None
        self._depth_tf_ok = False
        # Round 12 Y2/Y3: table plane fit + per-pixel plane heights.
        self.table_plane = None
        self._plane_values = None
        # Round 14 CC.3: vendor hand-to-camera transform (calibration.py
        # L28-33). Static seed; Round 12 Y1 composes the depth->color TF
        # into it once that arrives so this matrix is what vendor would
        # compute. Stored at instance scope so the world-position path
        # reads it without re-deriving every tick.
        self.hand2cam_tf_matrix = np.array([
            [0.0,  0.0, 1.0, -0.101],
            [-1.0, 0.0, 0.0,  0.0],
            [0.0, -1.0, 0.0,  0.05],
            [0.0,  0.0, 0.0,  1.0]], dtype=np.float64)
        # v5.1 FIX (B3): subscribe to BOTH candidate depth topics rather than
        # committing to the first. create_subscription never fails for a
        # non-publishing topic, so the old `break` always locked onto
        # depth_to_color even when only the raw topic published -> depth stayed
        # dead with no fallback. Now whichever topic actually DELIVERS frames
        # wins (see _depth_callback), upgrading to the colour-aligned
        # depth_to_color stream if it ever appears.
        for topic in ('/depth_cam/depth_to_color', '/depth_cam/depth/image_raw'):
            try:
                self.create_subscription(
                    Image, topic,
                    functools.partial(self._depth_callback, src_topic=topic),
                    1, callback_group=self.cam_group)
                _stage('init', f'depth subscribed (candidate): {topic}')
            except Exception as e:
                _stage('init', f'depth subscription failed for {topic}', exc=e)
        # Depth camera_info subscription so the plane fit has fx/fy/cx/cy.
        # v5.1 FIX: use VOLATILE (default) QoS, NOT TRANSIENT_LOCAL. The vendor
        # (shape_recognition.py:203 - message_filters.Subscriber, default QoS =
        # RELIABLE/VOLATILE) receives this exact topic fine on this hardware,
        # which proves the publisher is VOLATILE. A TRANSIENT_LOCAL *subscriber*
        # is QoS-INCOMPATIBLE with a VOLATILE publisher and receives NOTHING -
        # that was the real cause of the persistent "no_depth_caminfo_yet" (the
        # Round 17 PP.3 change assumed a latched/TRANSIENT_LOCAL publisher, which
        # the hardware contradicts). VOLATILE is compatible with BOTH a VOLATILE
        # and a TRANSIENT_LOCAL publisher, and camera_info is republished every
        # frame, so nothing is missed.
        try:
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   DurabilityPolicy, HistoryPolicy)
            qos_caminfo = QoSProfile(
                depth=10,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info',
                                     self._depth_cam_info_callback,
                                     qos_caminfo, callback_group=self.cam_group)
        except Exception as e:
            _stage('init', 'depth camera_info subscription failed', exc=e)
        # TF lookup runs in a background thread so we don't block boot if the
        # static publisher is slow.
        threading.Thread(target=self._lookup_depth_color_tf,
                         daemon=True).start()

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
        applied (or empty).

        default.yaml is the SINGLE boot source of truth. Every tuner-UI save
        (per-tab Save & Apply, Save ALL as default) merges into default.yaml,
        so it always holds the latest engine + knobs + everything else. The
        old yolo.yaml overlay was removed: the installer shipped a stale sample
        yolo.yaml that overwrote the user's saved model keys on every relaunch
        (conf/iou/max_det/engine reverting), which is the opposite of what we
        want. yolo.yaml is now ignored at boot."""
        params = {}
        if DEFAULT_PROFILE_PATH.exists():
            params.update(load_profile_yaml(DEFAULT_PROFILE_PATH))
        if not params:
            return {}
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
            # Task override change: push to worker and force-reload so the
            # next YOLO() call honours it.
            if p.name == 'engine_task' and isinstance(p.value, str):
                if hasattr(self, 'inference') and self.inference is not None:
                    self.inference.set_task_override(p.value)
                    self.inference.request_engine_swap(
                        self.p('engine_path'), force=True)
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
            # Class filter: apply the enabled-classes list to the worker the
            # moment it changes (Detection-tab Save & Apply sets this param),
            # so the filter takes effect without waiting for an engine reload.
            if p.name == 'yolo_enabled_classes' \
                    and hasattr(self, 'inference') and self.inference is not None:
                try:
                    classes = json.loads(p.value or '[]')
                    if isinstance(classes, list):
                        self.inference.set_yolo_knobs(classes=classes)
                        _stage('param', f'yolo_enabled_classes -> {classes} APPLIED')
                except Exception as e:
                    _stage('param', 'yolo_enabled_classes parse failed', exc=e)
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

    def _merge_into_default(self, keys):
        """Merge the current values of `keys` into default.yaml without
        disturbing the other persisted settings. This is what makes a per-tab
        "Save & Apply" survive relaunch - the node loads default.yaml at boot,
        so writing just the tab's keys here is enough. Returns True on success.
        """
        try:
            _ensure_profiles_dir()
            merged = load_profile_yaml(DEFAULT_PROFILE_PATH) if \
                DEFAULT_PROFILE_PATH.exists() else {}
            for k in keys:
                try:
                    merged[k] = self.p(k)
                except Exception:
                    pass
            save_profile_yaml(DEFAULT_PROFILE_PATH, merged)
            _stage('svc', f'merged {list(keys)} into {DEFAULT_PROFILE_PATH}')
            return True
        except Exception as e:
            _stage('svc', f'merge into {DEFAULT_PROFILE_PATH} failed', exc=e)
            return False

    # ------------------------------------------------------------------ misc state

    def _init_state(self):
        self.heart = None
        self.target_miss_count = 0
        self.transport_info = None
        self.start_transport = False
        # v5.1 (thread-safety): guards the start_transport / transport_info /
        # target handoff between the detection (sorting_loop) and motion
        # (transport_thread) threads and the enable/exit service callbacks.
        # Held ONLY for the brief field assignments - never across motion or
        # IO - so it cannot deadlock. The vendor uses an RLock for this handoff.
        # Created EXACTLY once: _init_state() re-runs on every ~/enter while the
        # transport/sorting threads are already alive (started in _startup), so
        # a plain reassignment here would swap in a NEW lock object mid-flight
        # and briefly break mutual exclusion. Guard so the lock identity is
        # stable across re-enters.
        if not hasattr(self, '_transport_lock'):
            self._transport_lock = threading.Lock()
        self.enable_sorting = False
        # YOLO knob changes received while sorting is ON are stashed here
        # and flushed when sorting toggles back OFF.
        self._pending_yolo_knobs = {}
        # Last calibrate result for the heartbeat mirror.
        self._last_calibrate = None
        # Last grip outcome (heartbeat mirror -> Grip-tab live label).
        # Keys: ts, label, outcome, final_pulse, peak_temp, duration_ms, source.
        self._last_grip = None
        # Test-grip in-progress guard so ~/test_grip won't reenter.
        self._test_grip_running = False
        # YOLO classes with no place target (set in _on_engine_loaded).
        self._unmapped_classes = []
        # Calibration in-progress guard.
        self._calibrating = False
        self.white_area_center = None
        self.enter = False
        self.roi = []
        self.count_move = 0
        self.count_still = 0
        self.target = None
        self.start_get_roi = False
        self.last_position = None
        # Round 17 NN.1: pixel of the currently-locked target's centroid,
        # used to disambiguate when label-only matching finds multiple
        # candidates of the same colour.
        self._last_target_pixel = None
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
            task = ''
            class_names = {}
            if hasattr(self, 'inference') and self.inference is not None:
                engine = os.path.basename(self.inference._engine_path or '')
                task = getattr(self.inference, '_task', None) or ''
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
                'task': task,
                'task_override': self.p('engine_task'),
                # Depth subscription status (Round 11). UI uses this to show
                # whether per-object Z is live (ON), missing (OFF), or stale.
                'depth_topic': getattr(self, '_depth_topic', ''),
                'depth_available': bool(getattr(self, '_depth_available', False)),
                'depth_age_ms': (int(1000 * (time.time() - getattr(self, '_latest_depth_ts', 0.0)))
                                 if getattr(self, '_depth_available', False) else -1),
                # Round 12 Y6: extra status for the Calibrate-tab panel.
                'depth_tf_ok': bool(getattr(self, '_depth_tf_ok', False)),
                'table_plane': (list(self.table_plane)
                                if getattr(self, 'table_plane', None) is not None
                                else None),
                # Round 14 CC.1: confirm vendor utils are importable so the
                # world-position math can use the vendor pipeline.
                'vendor_utils_ok': bool(_VENDOR_UTILS_OK),
                'last_calibrate': getattr(self, '_last_calibrate', None),
                # v5: class names from model.names (id -> name). The tuner
                # UI auto-populates the Classes filter + Places tab when
                # this changes.
                'class_names': {str(k): v for k, v in class_names.items()},
                # v5.1 FIX (A7): removed a duplicate 'last_calibrate' key here
                # (the dict already sets it above via getattr, the safe form).
                # The duplicate's bare self._last_calibrate would raise if the
                # attr were ever unset, and AttributeError there drops the
                # ENTIRE heartbeat publish, blanking the whole UI status panel.
                'last_grip': self._last_grip,
                'test_grip_running': bool(getattr(self, '_test_grip_running', False)),
                # v5.1 bin-teach: current commanded teach pose for the Bin Teach
                # tab's live readout (None until the first jog/goto).
                'teach_pose': (list(self._teach_pose)
                               if getattr(self, '_teach_pose', None) is not None
                               else None),
                'teach_running': bool(getattr(self, '_teach_running', False)),
                # v5.1 bin-teach (joint jog + workspace): live servo pulses,
                # the staged workspace centre, the edge count, and the last
                # teach status string for the Bin Teach tab.
                'teach_joints': (list(self._teach_joints)
                                 if getattr(self, '_teach_joints', None) is not None
                                 else None),
                'teach_center': (list(self._teach_center)
                                 if getattr(self, '_teach_center', None) is not None
                                 else None),
                'teach_edges': len(getattr(self, '_teach_edges', []) or []),
                'last_teach_msg': str(getattr(self, '_last_teach_msg', '') or ''),
                # Classes the loaded model can detect but that have no
                # place target configured (UI badges these).
                'unmapped_classes': list(getattr(self, '_unmapped_classes', [])),
                'unmapped_count': len(getattr(self, '_unmapped_classes', [])),
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
        # v5.1 FIX (B1): capture the home-config endpoint pose once, on its own
        # thread, so the vendor depth world-position path has the real
        # arm-base->hand transform (was always identity). Daemon + cached;
        # falls back to the legacy calibrated path until/if it succeeds.
        threading.Thread(
            target=lambda: self._capture_endpoint_pose([500, 520, 210, 50, 500]),
            daemon=True, name='endpoint-capture').start()
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

    def _capture_endpoint_pose(self, joint_angle, retries=12, retry_dt=1.0):
        """v5.1 FIX (B1 - critical): capture the forward-kinematics endpoint
        pose of the fixed observe/home joint config and stash it on the
        MotionController as `last_endpoint_pose`.

        The vendor world-position math (`utils.calculate_world_position`,
        utils.py:294) computes the pick point as
        `(endpoint + pose_end)[:3, 3]`, i.e. it ADDS the endpoint translation
        (arm-base -> hand) to the camera-frame point. v5 read
        `getattr(self.motion, 'last_endpoint_pose', None)`, which was NEVER
        set anywhere, so it always fell back to an identity matrix -> the
        arm-base offset was dropped and every depth-path pick was computed as
        if the camera/hand were at the world origin (lands off to the side).

        The observe config is constant, so we compute this ONCE, cached. Run
        from a dedicated daemon thread (see _startup) so the blocking FK
        service wait never sits on the ROS executor thread. Faithful to the
        vendor `shape_recognition.go_home` endpoint capture."""
        client = getattr(self, 'set_joint_value_target_client', None)
        if client is None:
            _stage('init', 'no set_joint_value_target client - endpoint pose '
                           'unavailable; world-pos uses legacy calibrated path')
            return
        for _ in range(max(1, int(retries))):
            if getattr(self.motion, 'last_endpoint_pose', None) is not None:
                return
            try:
                msg = set_joint_value_target(list(joint_angle))
                future = client.call_async(msg)
                res = self.motion._await_future(future, timeout=3.0)
                if res is not None and getattr(res, 'pose', None) is not None:
                    pose_t = res.pose.position
                    pose_r = res.pose.orientation
                    self.motion.last_endpoint_pose = common.xyz_quat_to_mat(
                        [pose_t.x, pose_t.y, pose_t.z],
                        [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
                    _stage('init', 'captured endpoint pose t=[%.4f, %.4f, %.4f]'
                                   % (pose_t.x, pose_t.y, pose_t.z))
                    return
            except Exception as e:
                _stage('init', 'endpoint pose capture attempt failed', exc=e)
            time.sleep(float(retry_dt))
        _stage('init', 'endpoint pose NOT captured after retries - vendor '
                       'depth world-pos path will fall back to the legacy '
                       'calibrated path (still functional, less depth-accurate)')

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
            # Round 12 Y2: load table plane if present (set by vendor or
            # the depth plane refit when depth is available).
            if 'plane' in config and config['plane'] is not None:
                self.table_plane = np.array(config['plane'],
                                            dtype=np.float64).ravel()
                _stage('roi', f'  loaded plane: {self.table_plane.tolist()}')
            else:
                self.table_plane = None
                self._plane_values = None
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
            with self._transport_lock:
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

    def _on_calib_finish(self, msg):
        if getattr(msg, 'data', False):
            self._calib_finished.set()

    def recalibrate_srv_callback(self, request, response):
        # Alias of ~/run_calibration so the existing UI client keeps working.
        return self.run_calibration_srv_callback(request, response)

    def run_calibration_srv_callback(self, request, response):
        # Spawn a worker and return immediately so the service reply isn't
        # held for the whole multi-second AprilTag sequence.
        if getattr(self, '_calibrating', False):
            _stage('svc', 'run_calibration ignored - already calibrating')
            response.success = False
            return response
        if not self.calib_enter_cli.wait_for_service(timeout_sec=2.0):
            _stage('calibrate', 'vendor calibration node not running - is '
                                'calibration_node.launch.py included? Aborting.')
            self._last_calibrate = {'ts': time.time(), 'ok': False,
                                    'error': 'no_vendor_node'}
            response.success = False
            return response
        self._calibrating = True
        threading.Thread(target=self._run_vendor_calibration, daemon=True).start()
        response.success = True
        return response

    def _call_trigger(self, client, timeout=5.0):
        """Call a std_srvs/Trigger client off the executor thread and poll
        its future (this runs on our own worker thread)."""
        fut = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        return bool(res and res.success)

    # Round 15: LAB color calibration is delegated to the vendor
    # lab_manager_node. v5 no longer hosts the LAB service surface.


    def _depth_plane_refit_srv(self, request, response):
        """Round 12 Y6 / Round 17 PP.2: refit the table plane from the
        latest depth frame and persist it to transform.yaml. Echoes the
        specific failure reason on response.message so the UI shows it."""
        try:
            plane, reason = self._fit_table_plane()
            if plane is None:
                _stage('calibrate', f'plane refit failed: {reason}')
                response.success = False
                response.message = reason
                return response
            cfg_path = os.path.join(self.config_path, self.config_file)
            try:
                with open(cfg_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            except Exception:
                data = {}
            data['plane'] = list(plane)
            try:
                with open(cfg_path, 'w') as f:
                    yaml.safe_dump(data, f)
                self.start_get_roi = True
                _stage('calibrate', 'plane refit: persisted')
                response.success = True
                response.message = 'ok'
            except Exception as e:
                _stage('calibrate', 'plane refit: yaml write failed', exc=e)
                response.success = False
                response.message = f'write_failed:{type(e).__name__}'
        except Exception as e:
            _stage('calibrate', 'plane refit crashed', exc=e)
            response.success = False
            response.message = f'exception:{type(e).__name__}'
        return response

    def _run_vendor_calibration(self):
        """Drive the vendor AprilTag calibration node end to end, then
        rebuild ROI from the transform.yaml it writes. Fully guarded -
        never raises, always tries to leave the vendor node in 'exit'.

        Round 15: vendor calibration_node is the ONLY path. No v5-side
        fallback - if the vendor node isn't reachable we surface that
        reason rather than running a less-faithful reimplementation."""
        try:
            self.enable_sorting = False
            try:
                self.motion.abort(True)
            except Exception:
                pass
            self.target = None
            _stage('calibrate', 'sorting force-stopped; starting calibration')
            self._calib_finished.clear()

            if not self.calib_enter_cli.wait_for_service(timeout_sec=2.0):
                _stage('calibrate', 'vendor calibration_node service not '
                                    'reachable - is it launched?')
                self._last_calibrate = {'ts': time.time(), 'ok': False,
                                        'source': 'vendor',
                                        'error': 'no_vendor_node'}
                return
            if not self._call_trigger(self.calib_enter_cli):
                _stage('calibrate', 'vendor enter failed/timed out')
                self._last_calibrate = {'ts': time.time(), 'ok': False,
                                        'source': 'vendor',
                                        'error': 'enter_failed'}
                self._call_trigger(self.calib_exit_cli, timeout=3.0)
                return
            if not self._call_trigger(self.calib_start_cli):
                _stage('calibrate', 'vendor start failed/timed out')
                self._last_calibrate = {'ts': time.time(), 'ok': False,
                                        'source': 'vendor',
                                        'error': 'start_failed'}
                self._call_trigger(self.calib_exit_cli, timeout=3.0)
                return
            # v5.1 FIX (A5): 20s was marginal. The vendor calibration_proc can
            # legitimately take longer on success: arm-to-pose settle (~1.5s) +
            # up to ~10s tag collection + solvePnP + a synchronous
            # /kinematics/get_current_pose round-trip + SearchPlane RANSAC,
            # THEN it publishes /calibration/finish. A short wait timed out and
            # called exit, aborting a calibration that would have succeeded.
            got = self._calib_finished.wait(timeout=35.0)
            if not got:
                _stage('calibrate', 'timed out waiting for /calibration/finish '
                                    '(no AprilTag in view? tag id must be '
                                    '1/2/3/100; tag must be steady)')
            self._call_trigger(self.calib_exit_cli, timeout=3.0)
            if got:
                self.start_get_roi = True
                _stage('calibrate', 'finished; rebuilding ROI from new '
                                    'transform.yaml')
                self._flash_overlay()
            self._last_calibrate = {'ts': time.time(), 'ok': bool(got),
                                    'source': 'vendor',
                                    'error': None if got else 'finish_timeout'}
        except Exception as e:
            _stage('calibrate', 'calibration crashed', exc=e)
            try:
                self._call_trigger(self.calib_exit_cli, timeout=3.0)
            except Exception:
                pass
            self._last_calibrate = {'ts': time.time(), 'ok': False,
                                    'source': 'vendor',
                                    'error': f'exception:{type(e).__name__}'}
        finally:
            self._calibrating = False

    def _flash_overlay(self):
        """Flash the calibration overlay for `calibrate_flash_secs` then
        revert to 'off'. Shown after a successful vendor calibrate."""
        try:
            self.set_parameters([rclpy.parameter.Parameter(
                'calibrate_overlay_mode', value='flash')])
            flash_secs = float(self.p('calibrate_flash_secs'))

            def _clear_flash():
                time.sleep(max(0.5, flash_secs))
                try:
                    self.set_parameters([rclpy.parameter.Parameter(
                        'calibrate_overlay_mode', value='off')])
                except Exception:
                    pass
            threading.Thread(target=_clear_flash, daemon=True).start()
        except Exception as e:
            _stage('calibrate', 'flash overlay schedule failed', exc=e)


    def _fit_table_plane(self):
        """Round 12 Y2 / Round 17 PP.1: RANSAC plane fit on the latest depth
        frame via vendor SearchPlane. Returns (plane, reason). `plane` is the
        [a,b,c,d] list on success, None on failure; `reason` is a short
        token the UI can show to the operator instead of an opaque 'failed'."""
        if not getattr(self, '_depth_available', False):
            return None, 'no_depth_subscription'
        if self._latest_depth is None:
            return None, 'no_depth_frame_yet'
        try:
            from app.utils.search_plane import SearchPlane
        except Exception:
            try:
                from utils.search_plane import SearchPlane
            except Exception as e:
                _stage('calibrate', 'plane fit: SearchPlane not importable', exc=e)
                return None, 'searchplane_unavailable'
        try:
            depth_info = getattr(self, '_depth_cam_info', None)
            if depth_info is None:
                return None, 'no_depth_caminfo_yet'
            p = depth_info.p if hasattr(depth_info, 'p') else depth_info.k
            fx = p[0]; fy = p[5] if len(p) >= 12 else depth_info.k[4]
            cx = p[2]; cy = p[6] if len(p) >= 12 else depth_info.k[5]
            H = self._latest_depth.shape[0]
            W = self._latest_depth.shape[1]
            searcher = SearchPlane(W, H, [fx, fy, cx, cy])
            _, plane, _ = searcher.find_plane(self._latest_depth)
            if plane is None:
                return None, 'searchplane_returned_none'
            arr = np.asarray(plane, dtype=np.float64).ravel().tolist()
            _stage('calibrate', f'plane fit: {arr}')
            return arr, 'ok'
        except Exception as e:
            _stage('calibrate', 'plane fit failed', exc=e)
            return None, f'exception:{type(e).__name__}'

    def _on_engine_loaded(self, path):
        """InferenceWorker callback: fires from the worker thread after a
        successful engine load (initial or hot-swap). Mirrors model.names
        into target_labels so the tuner UI can render per-class checkboxes,
        applies the YOLO classes filter from the saved config, and flags
        any class that has no place target so the operator knows it won't
        be sortable until they set one."""
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
                # Validate place coverage: a detected class with no place
                # target will be refused at drop time (_do_place returns
                # False). Surface it loudly + on the heartbeat for the UI.
                try:
                    user_pp = json.loads(self.p('place_positions') or '{}')
                    place_keys = set(user_pp.keys()) if isinstance(user_pp, dict) else set()
                except Exception:
                    place_keys = set()
                place_keys |= set(self.DEFAULT_PLACE_POSITIONS.keys())
                self._unmapped_classes = sorted(
                    n for n in names.values() if n not in place_keys)
                for n in self._unmapped_classes:
                    _stage('engine-load', f'class {n!r} has NO place target - '
                                          f'_do_place will refuse it; set one in '
                                          f'the Places tab')
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

    def test_grip_srv_callback(self, request, response):
        """BETA: interactive per-class grip test. data_str = YOLO class
        name (used to look up the per-class max-strength cap and as the
        label on the telemetry record). data_bool is unused.

        Spawns a worker and returns immediately so the service reply
        isn't held while the operator places the object."""
        label = (request.data_str or '').strip() or 'unknown'
        if self.enable_sorting:
            _stage('svc', 'test_grip refused - stop sorting first')
            response.success = False
            return response
        if getattr(self, '_calibrating', False):
            _stage('svc', 'test_grip refused - calibration in progress')
            response.success = False
            return response
        if getattr(self, '_test_grip_running', False):
            _stage('svc', 'test_grip refused - already running')
            response.success = False
            return response
        if self.kinematics_client is None:
            _stage('svc', 'test_grip refused - kinematics not available')
            response.success = False
            return response
        self._test_grip_running = True
        threading.Thread(target=self._run_test_grip, args=(label,),
                         daemon=True).start()
        response.success = True
        return response

    def _run_test_grip(self, label):
        """Worker thread: drive arm to a safe test pose, open jaws, dwell
        so the operator places the object, run compliance_grasp with this
        class's strength cap, hold briefly, release. Telemetry lands on
        self._last_grip which the heartbeat publishes."""
        try:
            speed = max(0.1, 1.0 / float(self.p('motion_speed')))
            aggression = float(self.p('aggression'))
            open_pulse = int(self.p('gripper_open_pulse'))
            close_pulse = int(self.p('gripper_close_pulse'))
            max_strength = int(self._grasp_strength_for(label, close_pulse))
            grasp_step = int(self.p('grasp_step_pulse'))
            grasp_dwell = float(self.p('grasp_step_dwell'))
            grasp_stall = int(self.p('grasp_stall_pulse'))
            grasp_timeout = float(self.p('grasp_timeout'))
            grasp_max_temp = int(self.p('grasp_max_temp'))
            test_dwell = float(self.p('test_grip_dwell'))

            _stage('test-grip', f'class={label!r} max_strength={max_strength} '
                                f'dwell={test_dwell:.1f}s starting')
            # 1. Park home + open gripper so the jaws are clear.
            self.go_home(False)
            self.motion.set_gripper(open_pulse, 0.3 * speed)
            # 2. Move to a fixed test pose: above the workspace, jaws down.
            #    Reachable for the JetArm; doesn't depend on calibration.
            test_pose = [0.0, 0.20, 0.10]
            if self.motion.goto_pose(test_pose, 80,
                                     duration=max(0.6, 1.0 * speed / aggression),
                                     parallel_base=True,
                                     pitch_range=(-90.0, 90.0)) is None:
                _stage('test-grip', 'test-pose IK failed')
                self._last_grip = {'ts': time.time(), 'label': label,
                                   'outcome': 'ik_failed', 'source': 'test'}
                return
            # 3. Dwell so the operator places the object.
            _stage('test-grip', f'place an object between the jaws ({test_dwell:.1f}s) ...')
            self.motion._sleep(test_dwell)
            # 4. Compliance grasp with this class's cap.
            t0 = time.time()
            outcome = self.motion.compliance_grasp(
                target_pulse=max_strength, open_pulse=open_pulse,
                step=grasp_step, dwell=grasp_dwell,
                stall_thresh=grasp_stall, timeout=grasp_timeout,
                max_temp=grasp_max_temp)
            grip_ms = int(1000 * (time.time() - t0))
            # 5. Hold briefly so the user can eyeball the grip.
            self.motion._sleep(1.5)
            # 6. Telemetry snapshot.
            final_pulse = None; peak_temp = None
            try:
                st = self.motion.get_servo_state(
                    GRIPPER_ID, fields=('position', 'temperature'))
                if st.get('position') is not None: final_pulse = int(st['position'])
                if st.get('temperature') is not None: peak_temp = int(st['temperature'])
            except Exception:
                pass
            self._last_grip = {
                'ts': time.time(), 'label': label, 'outcome': outcome,
                'final_pulse': final_pulse, 'peak_temp': peak_temp,
                'duration_ms': grip_ms, 'source': 'test',
            }
            _stage('test-grip', f'{label} -> {outcome} pulse={final_pulse} '
                                f'temp={peak_temp} {grip_ms}ms')
            # 7. Release and home.
            self.motion.set_gripper(open_pulse, 0.3 * speed)
            self.motion._sleep(0.3)
            self.go_home(False)
        except Exception as e:
            _stage('test-grip', 'crashed - opening gripper + going home', exc=e)
            try:
                self.motion.set_gripper(int(self.p('gripper_open_pulse')), 0.3)
                self.go_home(True)
            except Exception:
                pass
            self._last_grip = {'ts': time.time(), 'label': label,
                               'outcome': 'crashed', 'source': 'test'}
        finally:
            self._test_grip_running = False

    # ------------------------------------------------------------------ bin-teach
    def _teach_log(self, msg):
        """Mirror a teach status to the stage log, the heartbeat
        (_last_teach_msg), and the low-latency ~/teach_status topic so the Bin
        Teach tab updates immediately after each action."""
        self._last_teach_msg = str(msg)
        _stage('teach', msg)
        self._publish_teach_status()

    def _publish_teach_status(self):
        """Publish the current teach readout on ~/teach_status (instant UI
        feedback; never raises)."""
        try:
            self.teach_status_publisher.publish(String(data=json.dumps({
                'teach_pose': (list(self._teach_pose)
                               if getattr(self, '_teach_pose', None) is not None
                               else None),
                'teach_joints': (list(self._teach_joints)
                                 if getattr(self, '_teach_joints', None) is not None
                                 else None),
                'teach_center': (list(self._teach_center)
                                 if getattr(self, '_teach_center', None) is not None
                                 else None),
                'teach_edges': len(getattr(self, '_teach_edges', []) or []),
                'teach_running': bool(getattr(self, '_teach_running', False)),
                'last_teach_msg': str(getattr(self, '_last_teach_msg', '') or ''),
                # current per-class bin map so the UI's "Saved bins" list (with
                # a Go button each) stays live after every save.
                'place_positions': self.p('place_positions'),
            })))
        except Exception:
            pass

    def _fk_world_pose(self, pulses):
        """Forward-kinematics of 5 servo pulses -> world [x, y, z] in the
        arm-base frame (the SAME frame as place_positions and the IK target -
        verified: FK service and IK service share one kinematic chain). This is
        how the joint-jog readout gets the gripper's real world coordinate
        without a get_current_pose client. Worker-thread only (blocking await).
        Returns None on failure."""
        client = getattr(self, 'set_joint_value_target_client', None)
        if client is None:
            return None
        try:
            msg = set_joint_value_target([int(p) for p in pulses])
            res = self.motion._await_future(client.call_async(msg), timeout=3.0)
            if res is not None and getattr(res, 'pose', None) is not None:
                p = res.pose.position
                return [float(p.x), float(p.y), float(p.z)]
        except Exception as e:
            _stage('teach', 'FK world readout failed', exc=e)
        return None

    def _teach_seed_pose(self):
        """Seed the world-jog pose. This arm can't report servo positions, so
        prefer FK of the tracked joints (set by Home / a prior move); fall back
        to the captured observe endpoint, then a safe central guess."""
        if getattr(self, '_teach_joints', None) is not None:
            wp = self._fk_world_pose(self._teach_joints)
            if wp is not None:
                return wp
        ep = getattr(self.motion, 'last_endpoint_pose', None)
        if ep is not None:
            try:
                return [float(ep[0, 3]), float(ep[1, 3]), float(ep[2, 3])]
            except Exception:
                pass
        return [0.12, 0.0, 0.18]

    def teach_srv_callback(self, request, response):
        """v5.1 manual bin-teach + workspace teach. JSON command in data_str:
          {"action":"joint_jog","joint":3,"delta":10}  -> jog ONE servo by pulses
          {"action":"read"}                            -> refresh joints+world readout
          {"action":"jog","dx":1,"dy":0,"dz":0,"step":0.01} -> jog world X/Y/Z (IK)
          {"action":"goto","class":"red cube"}         -> move to that saved bin
          {"action":"home"}                            -> safe central pose
          {"action":"save","class":"red cube"}         -> save the readout as that bin
          {"action":"set_center"}                      -> stage the workspace centre
          {"action":"add_edge"}                        -> stage a workspace edge
          {"action":"clear_workspace"}                 -> drop staged centre+edges
          {"action":"save_workspace","force":false}    -> re-anchor world origin
        data_bool = persist (place_positions / workspace_size to default.yaml).
        A bin coordinate is just the bin's LOCATION - the arm computes its own
        drop approach; taught bins skip the vision-reach affine so the cube
        lands exactly at the saved coordinate. Arm-moving actions run on a worker
        thread (set_servo / goto_pose must run off the executor); save and the
        workspace actions are synchronous. Refused while sorting/calibrating/a
        transport is live."""
        import json
        try:
            cmd = json.loads(request.data_str or '{}')
            if not isinstance(cmd, dict):
                cmd = {}
        except Exception:
            cmd = {}
        action = str(cmd.get('action', '')).strip()
        if self.enable_sorting:
            self._teach_log('refused - stop sorting first')
            response.success = False; return response
        if getattr(self, '_calibrating', False) or self.start_transport:
            self._teach_log('refused - calibration/transport in progress')
            response.success = False; return response
        if action == 'save':
            if getattr(self, '_teach_running', False):
                self._teach_log('save refused - a jog is still moving; wait')
                response.success = False; return response
            cls = str(cmd.get('class', '')).strip()
            tp = getattr(self, '_teach_pose', None)
            if not cls or tp is None:
                self._teach_log(f'save refused - need a class and a readout '
                                f'(class={cls!r}, pose={tp})')
                response.success = False; return response
            try:
                pp = json.loads(self.p('place_positions') or '{}')
                if not isinstance(pp, dict):
                    pp = {}
                # 4th element 'taught' marks this as a hand-set coordinate so
                # _do_place drives straight to it (skips the vision-reach affine).
                pp[cls] = [round(float(tp[0]), 4), round(float(tp[1]), 4),
                           round(float(tp[2]), 4), 'taught']
                self.set_parameters([rclpy.parameter.Parameter(
                    'place_positions', value=json.dumps(pp))])
                self.place_position[cls] = [float(tp[0]), float(tp[1]),
                                            float(tp[2])]  # keep fallback in sync
                if request.data_bool:
                    self._merge_into_default(['place_positions'])
                self._teach_log(f'SAVE {cls} = {pp[cls]} (persist={request.data_bool})')
                response.success = True
            except Exception as e:
                self._teach_log(f'save failed: {type(e).__name__}')
                _stage('teach', 'save failed', exc=e)
                response.success = False
            return response
        # workspace teach (centre/edges) -> synchronous (uses the current
        # readout + a transform.yaml write; no arm motion of its own).
        if action in ('set_center', 'add_edge', 'clear_workspace', 'save_workspace'):
            if getattr(self, '_teach_running', False):
                self._teach_log(f'{action} refused - a jog is still moving; wait')
                response.success = False; return response
            try:
                ok, msg = self._teach_workspace(action, cmd, bool(request.data_bool))
                self._teach_log(f'{action}: {msg}')
                response.success = bool(ok)
            except Exception as e:
                self._teach_log(f'{action} failed: {type(e).__name__}')
                _stage('teach', 'workspace teach crashed', exc=e)
                response.success = False
            return response
        # jog / joint_jog / read / goto / home -> move (or read) on a worker
        # thread; the service calls + goto_pose must run OFF the executor.
        if action not in ('jog', 'joint_jog', 'read', 'goto', 'home'):
            self._teach_log(f'unknown teach action {action!r}')
            response.success = False; return response
        if action in ('jog', 'goto', 'home') and self.kinematics_client is None:
            self._teach_log('refused - kinematics (IK) not available')
            response.success = False; return response
        if getattr(self, '_teach_running', False):
            self._teach_log('busy - previous teach move still running')
            response.success = False; return response
        self._teach_running = True
        threading.Thread(target=self._run_teach, args=(action, cmd),
                         daemon=True).start()
        response.success = True
        return response

    def _run_teach(self, action, cmd):
        """Worker for the arm-moving / readout teach actions. World jog/goto/home
        use IK (goto_pose) and track the commanded pose in self._teach_pose;
        joint_jog commands ONE servo and reads the resulting world pose back via
        FK; read refreshes the joints+world readout without moving. All update
        self._teach_pose (the coordinate 'save' captures) and self._teach_joints."""
        try:
            speed = max(0.1, 1.0 / float(self.p('motion_speed')))
            aggression = float(self.p('aggression'))

            # ---- joint-by-joint jog: command ONE servo (OPEN-LOOP) ----------
            # This arm's bus_servo/get_state never returns positions (the
            # standard grasp is open-loop too), so we TRACK the commanded servo
            # pulses in self._teach_joints instead of reading them. They are
            # seeded by Home (or a successful world goto/jog); the live world
            # readout comes from FK of the tracked pulses (FK works - it's how
            # the init endpoint pose is captured).
            if action == 'joint_jog':
                if self._teach_joints is None:
                    self._teach_log('joint_jog: joints not synced yet - press '
                                    'Home first (this arm can\'t report servo '
                                    'positions, so jogs are tracked from Home)')
                    return
                try:
                    jid = int(cmd.get('joint', 0))
                    delta = int(round(float(cmd.get('delta', 0))))
                except Exception:
                    jid, delta = 0, 0
                if jid < 1 or jid > 5:
                    self._teach_log(f'joint_jog: bad joint id {jid} (use 1..5)')
                    return
                joints = [int(p) for p in self._teach_joints]
                new = self.motion.set_servo(
                    jid, joints[jid - 1] + delta,
                    max(0.2, 0.5 * speed / aggression))
                joints[jid - 1] = new
                self._teach_joints = joints
                wp = self._fk_world_pose(joints)
                if wp is not None:
                    self._teach_pose = wp
                sign = ('+%d' % delta) if delta >= 0 else str(delta)
                self._teach_log(
                    f'joint_jog servo{jid} {sign} -> {new} pulse; '
                    f'joints={joints}; '
                    f'world={[round(v, 3) for v in (self._teach_pose or [])]}')
                return

            # ---- read-back: refresh the world readout from tracked joints ----
            if action == 'read':
                if self._teach_joints is None:
                    self._teach_log('read: joints not synced - press Home first')
                    return
                wp = self._fk_world_pose(self._teach_joints)
                if wp is not None:
                    self._teach_pose = wp
                self._teach_log(
                    f'read joints={self._teach_joints}; '
                    f'world={[round(v, 3) for v in (self._teach_pose or [])]}')
                return

            # ---- home: command the physical home servo config DIRECTLY -----
            # (always reachable; IK to a Cartesian guess like [0,0.2,0.1] has
            # no solution at pitch 80 and was failing every time).
            if action == 'home':
                # commands the physical home config AND syncs the tracked joints
                # to it, so joint jog has a known starting point afterwards.
                self.go_home(interrupt=False)
                self._teach_joints = [500, 520, 210, 50, 500]
                wp = self._fk_world_pose(self._teach_joints)
                if wp is not None:
                    self._teach_pose = wp
                self._teach_log(
                    f'home -> joints={self._teach_joints} '
                    f'world={[round(v, 3) for v in (self._teach_pose or [])]}')
                return

            # ---- world XYZ jog / goto via IK -------------------------------
            # Use the DEFAULT pitch range (-180,180) like _do_place (the
            # (-90,90) grab range was rejecting reachable drop points), and for
            # goto mirror _do_place's hover-then-descend so teach-goto reaches
            # exactly what placement reaches.
            dur = max(0.4, 0.8 * speed / aggression)
            if action == 'goto':
                cls = str(cmd.get('class', '')).strip()
                p = self._resolve_place_position(cls)
                if p is None:
                    self._teach_log(f'goto: no saved bin for {cls!r} yet')
                    return
                target = [float(p[0]), float(p[1]), float(p[2])]
                hover = [target[0], target[1], target[2] + 0.05]
                if self.motion.goto_pose(hover, 80, duration=dur,
                                         parallel_base=True) is None:
                    self._teach_log(f'goto {[round(v, 3) for v in target]} '
                                    f'hover UNREACHABLE (IK) - not moved')
                    return
                res = self.motion.goto_pose(
                    target, 80, duration=max(0.35, 0.6 * speed / aggression),
                    parallel_base=False)
                if res is None:
                    self._teach_pose = hover
                    self._teach_log(f'goto {[round(v, 3) for v in target]} '
                                    f'descend UNREACHABLE (IK) - left at hover')
                    return
                self._teach_pose = target
                try:
                    self._teach_joints = [int(round(v)) for v in res]
                except Exception:
                    pass
                self._teach_log(f'goto -> {[round(v, 3) for v in target]}')
                return

            # action == 'jog'
            if self._teach_pose is None:
                self._teach_pose = self._teach_seed_pose()
            step = abs(float(cmd.get('step', 0.01)))
            delta = [float(cmd.get('dx', 0.0)) * step,
                     float(cmd.get('dy', 0.0)) * step,
                     float(cmd.get('dz', 0.0)) * step]
            for i in range(3):
                self._teach_pose[i] += delta[i]
            self._teach_pose[2] = max(float(self.p('min_descend_z_m')),
                                      self._teach_pose[2])
            res = self.motion.goto_pose(list(self._teach_pose), 80,
                                        duration=dur, parallel_base=True)
            if res is None:
                self._teach_log(f'jog {[round(v, 3) for v in self._teach_pose]} '
                                f'UNREACHABLE (IK) - not moved')
                for i in range(3):
                    self._teach_pose[i] -= delta[i]
            else:
                try:
                    self._teach_joints = [int(round(v)) for v in res]
                except Exception:
                    pass
                self._teach_log(f'jog -> {[round(v, 3) for v in self._teach_pose]}')
        except Exception as e:
            self._teach_log(f'{action} failed: {type(e).__name__}')
            _stage('teach', 'move failed', exc=e)
        finally:
            self._teach_running = False
            self._publish_teach_status()

    def _teach_workspace(self, action, cmd, persist):
        """Teach the workspace CENTRE and EDGES to (re)anchor the world map.

        Writes ONLY white_area_pose_world's translation in transform.yaml (the
        world origin the legacy pick adds to every detection) from the taught
        centre, and sizes the operator overlay via the workspace_size_x/y
        params from the taught edges. It NEVER touches extristric / corners /
        plane - that camera geometry can only come from the AprilTag, so the
        AprilTag CALIBRATE must have run at least once first. Returns (ok, msg).
        """
        if not hasattr(self, '_teach_edges') or self._teach_edges is None:
            self._teach_edges = []
        tp = getattr(self, '_teach_pose', None)

        if action == 'clear_workspace':
            self._teach_center = None
            self._teach_edges = []
            return True, 'staged centre + edges cleared'

        if action == 'set_center':
            if tp is None:
                return False, 'no readout yet - jog to the mat centre, then Set Centre'
            self._teach_center = [float(tp[0]), float(tp[1]), float(tp[2])]
            return True, f'centre staged = {[round(v, 4) for v in self._teach_center]}'

        if action == 'add_edge':
            if tp is None:
                return False, 'no readout yet - jog to an edge first'
            self._teach_edges.append([float(tp[0]), float(tp[1]), float(tp[2])])
            return True, (f'edge #{len(self._teach_edges)} staged = '
                          f'{[round(v, 4) for v in self._teach_edges[-1]]}')

        if action == 'save_workspace':
            center = getattr(self, '_teach_center', None)
            if center is None:
                return False, 'set the centre first (jog to mat centre, Set Centre)'
            cfg_path = os.path.join(self.config_path, self.config_file)
            try:
                with open(cfg_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            except Exception as e:
                return False, (f'transform.yaml unreadable - run CALIBRATE '
                               f'(AprilTag) first ({type(e).__name__})')
            # We must NOT invent the camera geometry; require it to exist.
            if 'extristric' not in data or 'corners' not in data:
                return False, ('transform.yaml has no AprilTag geometry - run '
                               'CALIBRATE first (centre/edges only re-anchor it)')
            # guardrail: a centre far from the current origin is likely a mis-jog.
            dxy = 0.0
            try:
                old = np.array(data.get('white_area_pose_world'))[:3, 3]
                dxy = float(((center[0] - old[0]) ** 2 +
                             (center[1] - old[1]) ** 2) ** 0.5)
            except Exception:
                dxy = 0.0
            if dxy > 0.05 and not bool(cmd.get('force', False)):
                return False, (f'taught centre is {dxy * 100:.1f} cm from the '
                               f'current origin - re-send with Confirm to apply')
            # write ONLY the world origin (identity rotation + taught centre),
            # matching the vendor white_area_pose_world shape exactly.
            data['white_area_pose_world'] = [
                [1.0, 0.0, 0.0, float(center[0])],
                [0.0, 1.0, 0.0, float(center[1])],
                [0.0, 0.0, 1.0, float(center[2])],
                [0.0, 0.0, 0.0, 1.0]]
            try:
                with open(cfg_path, 'w') as f:
                    yaml.safe_dump(data, f)
            except Exception as e:
                return False, f'transform.yaml write failed ({type(e).__name__})'
            extra = ''
            if self._teach_edges:
                sx = max(abs(e[0] - center[0]) for e in self._teach_edges) * 2.0
                sy = max(abs(e[1] - center[1]) for e in self._teach_edges) * 2.0
                sx = max(0.05, min(1.0, sx))
                sy = max(0.05, min(1.0, sy))
                try:
                    self.set_parameters([
                        rclpy.parameter.Parameter('workspace_size_x', value=float(sx)),
                        rclpy.parameter.Parameter('workspace_size_y', value=float(sy))])
                    if persist:
                        self._merge_into_default(['workspace_size_x', 'workspace_size_y'])
                    extra = (f'; overlay {sx:.3f}x{sy:.3f} m from '
                             f'{len(self._teach_edges)} edge(s)')
                except Exception as e:
                    extra = f'; overlay size set FAILED ({type(e).__name__})'
            # reload ROI + white_area_center live from the rewritten file.
            self.start_get_roi = True
            return True, (f'world origin re-anchored to '
                          f'{[round(v, 4) for v in center]}{extra} '
                          f'(persist={persist})')

        return False, f'unknown workspace action {action!r}'

    def apply_and_persist_srv_callback(self, request, response):
        """Apply a JSON {param: value} map via set_parameters (which reuses
        _on_param_change: engine swaps, YOLO knobs land, JSON-string params
        like place_positions/grasp_strength/yolo_enabled_classes pass through)
        and, if data_bool, merge those keys into default.yaml so they survive
        relaunch. This single service backs every per-tab Save & Apply button.
        """
        _stage('svc', f'apply_and_persist (persist={request.data_bool})')
        try:
            cfg = json.loads(request.data_str or '{}')
        except Exception as e:
            _stage('svc', 'apply_and_persist: malformed JSON', exc=e)
            response.success = False
            return response
        if not isinstance(cfg, dict):
            _stage('svc', 'apply_and_persist: payload is not an object')
            response.success = False
            return response
        applied = []
        for k, v in cfg.items():
            try:
                self.set_parameters([rclpy.parameter.Parameter(k, value=v)])
                applied.append(k)
            except Exception as e:
                _stage('svc', f'  {k}={v!r} rejected', exc=e)
        if request.data_bool and applied:
            if not self._merge_into_default(applied):
                response.success = False
                return response
        _stage('svc', f'apply_and_persist applied {applied}')
        response.success = True
        return response

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
        accepted = ('engine_path', 'engine_task', 'yolo_conf_thresh',
                    'yolo_iou_thresh', 'yolo_max_det', 'inference_max_hz')
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
                if self.inference.request_engine_swap(cfg['engine_path']):
                    _stage('svc', f'engine swap requested -> {cfg["engine_path"]}')
            except Exception as e:
                _stage('svc', 'engine hot-swap from save failed', exc=e)

        # 5. Persist if requested. default.yaml is the single boot source, so
        #    we merge the model keys there. (We no longer write yolo.yaml - it
        #    is not read at boot; this service is legacy, the new UI uses
        #    apply_and_persist.)
        model_keys = ('engine_path', 'engine_task',
                      'yolo_conf_thresh', 'yolo_iou_thresh',
                      'yolo_max_det', 'inference_max_hz', 'yolo_enabled_classes')
        if request.data_bool:
            if not self._merge_into_default(model_keys):
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

    def reload_engine_srv_callback(self, request, response):
        """Re-init the YOLO model from the currently configured engine_path
        without changing the path. Refuses while CALIBRATE is in progress or
        a swap is already pending (those bail out cleanly; the user can
        retry). The worker services the reload between frames, same as a
        normal hot-swap, so inference pauses briefly but never races."""
        path = self.p('engine_path')
        _stage('svc', f'reload_engine -> {path}')
        if self._calibrating:
            _stage('svc', 'reload_engine refused - calibration in progress')
            response.success = False
            response.message = 'calibration in progress'
            return response
        if self.inference.swap_pending():
            _stage('svc', 'reload_engine refused - swap already pending')
            response.success = False
            response.message = 'engine swap already pending'
            return response
        ok = self.inference.request_engine_swap(path, force=True)
        response.success = ok
        response.message = f'reloading {path}' if ok else 'reload rejected'
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
            # If the preset carried an engine_path, _on_param_change above will
            # have queued the swap. Surface it so the UI/log shows the model
            # actually changing with the preset.
            if 'engine_path' in params:
                _stage('svc', f'  preset engine -> {params["engine_path"]}')
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
        # If depth is enabled + available, use the median depth at the pixel
        # as Z so the arm descends to the actual top of the object instead
        # of the hardcoded table-plane assumption. Falls back to `height`
        # when depth is unavailable / stale / sparse.
        try:
            if bool(self.p('use_depth_for_z')):
                z = self._depth_at(int(pixel[0]), int(pixel[1]))
                if z is not None:
                    height = float(z)
        except Exception:
            pass
        return self.get_object_world_position(pixel, self.intrinsic, self.extristric,
                                              self.white_area_center, height)

    # ------------------------------------------------------------------ vision

    def _detections_from_results(self, bgr_image, roi, yolo_results):
        """Run YOLO detection on the current frame and
        return target_info plus a primitives dict for the overlay renderer.

        Returns (target_info, primitives) where primitives is:
            {'yolo_ops': [(kind, *args), ...],  # 'rect' or 'circle'
             'color_corners': [np.intp array of 4 (x,y), ...]}
        No cv2 drawing happens in this method — the republisher applies the
        primitives to a fresh raw frame at 30Hz (see _draw_overlay).
        """
        target_info = []
        yolo_ops = []
        # NOTE: do not crop bgr_image to roi here - v5 runs YOLO on the FULL
        # frame in InferenceWorker (see L368), so detection coords are
        # already in full-frame pixel space. roi is used for ROI gating /
        # workspace bounds only, not for shifting detection coords.
        # Manual pixel nudge (default 0): shifts every detection so the
        # overlay boxes sit on the objects when calibration can't be redone.
        # Same coords feed the world projection, so this also corrects the
        # pixel used for picking.
        ox = int(self.p('detection_offset_x'))
        oy = int(self.p('detection_offset_y'))
        # Depth sanity gate config (Round 11). When depth is available, a
        # detection whose median depth is outside [min, max] gets dropped
        # (filters hands and floor reflections without retraining).
        use_depth = bool(self.p('use_depth_for_z'))
        dmin = float(self.p('depth_min_z_m'))
        dmax = float(self.p('depth_max_z_m'))

        def _depth_ok(px, py):
            if not use_depth:
                return True
            z = self._depth_at(int(px), int(py))
            if z is None:
                return True   # depth unavailable - don't reject
            return dmin <= z <= dmax
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
                    cx += ox; cy += oy
                    if not _depth_ok(cx, cy):
                        continue
                    # ultralytics OBB: r is radians normalized to [-45,135);
                    # `w` is the LONG side and r is the angle of `w` (confirmed
                    # against the ultralytics OBB task docs). The grasp clamps
                    # the SHORT axis (see _select_gripper_yaw). If the physical
                    # gripper is mounted 90 deg off the wrist zero, correct it
                    # at runtime with the grasp_yaw_offset_deg tunable rather
                    # than hand-editing here.
                    angle = int(math.degrees(r))
                    corners = obb.xyxyxyxy[0].cpu().numpy()
                    corners = [(int(x + ox), int(y + oy)) for x, y in corners]
                    target_info.append([label, 1, (int(cx), int(cy)),
                                        (int(w), int(h)), angle])
                    yolo_ops.append(('obb', corners, label))
            elif hasattr(result, 'boxes') and result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls.cpu().numpy()) if hasattr(box, 'cls') else 0
                    label = names.get(cls_id, f'cls_{cls_id}')
                    cx = (x1 + x2) / 2 + ox
                    cy = (y1 + y2) / 2 + oy
                    if not _depth_ok(cx, cy):
                        continue
                    w, h = x2 - x1, y2 - y1
                    target_info.append([label, 1, (int(cx), int(cy)),
                                        (int(w), int(h)), 0])
                    yolo_ops.append(('rect',
                                     (int(x1 + ox), int(y1 + oy)),
                                     (int(x2 + ox), int(y2 + oy)),
                                     label))
        primitives = {'yolo_ops': yolo_ops, 'color_corners': []}
        return target_info, primitives

    def _calibration_cfg(self):
        """calibration.yaml cached in memory, re-read only when the file
        mtime changes. Used by the pick/place paths and the live overlay
        (which calls the world math per-object per-tick)."""
        path = os.path.join(self.config_path, self.calibration_file)
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return getattr(self, '_calib_cache', {}) or {}
        if getattr(self, '_calib_cache_mtime', None) != mtime:
            self._calib_cache = common.get_yaml_data(path)
            self._calib_cache_mtime = mtime
        return self._calib_cache

    def _apply_world_offsets(self, position):
        """Apply the calibration.yaml pixel + kinematics affines + the
        live manual grip_offset and workspace_scale to a world position.
        Single source of truth shared by get_object_world_position (the
        real pick) and the calibration overlay, so the overlay can never
        drift from the pick.

        v5.1 FIX (double-kinematics): apply ONLY the `pixel` affine here, NOT
        `kinematics`. The vendor applies `pixel` in the world-position calc
        (object_sorting.get_object_world_position) and `kinematics` exactly
        ONCE in the transport thread (object_sorting.transport_thread). v5's
        transport_thread already applies the kinematics affine via
        _apply_kinematics_calibration, so the Round-14 BB.1 addition of the
        kinematics affine HERE double-applied it (scale squared, offset x2) -
        e.g. with kinematics y-scale 1.05 / y-offset 0.006 the pick landed
        ~+9 mm off in Y and the jaws closed beside the cube. Reverted to
        pixel-only so kinematics is applied exactly once, matching vendor."""
        cfg = self._calibration_cfg()
        # pixel affine (extrinsic-side correction). The kinematics affine is
        # applied ONCE downstream in transport_thread (_apply_kinematics_
        # calibration), matching the vendor split - do NOT also apply it here
        # (that double-applied it and pushed the grasp off-target).
        offset = tuple(cfg.get('pixel', {}).get('offset', (0.0, 0.0, 0.0)))
        scale = tuple(cfg.get('pixel', {}).get('scale', (1.0, 1.0, 1.0)))
        for i in range(3):
            position[i] = position[i] * scale[i] + offset[i]
        # Manual world nudge (default 0): shift the final pick position so
        # the arm lands on the object, compensating calibration drift when
        # the AprilTag can't be re-run. Read live so the slider applies on
        # the next pick / overlay tick.
        position[0] += float(self.p('grip_offset_x'))
        position[1] += float(self.p('grip_offset_y'))
        # Workspace scale: stretch/shrink around world origin AFTER the
        # offset so "shift by N m" means a fixed distance regardless of
        # scale. 1.0 = no change.
        ws = float(self.p('workspace_scale'))
        position[0] *= ws
        position[1] *= ws
        return position

    def get_object_world_position(self, position, intrinsic, extristric,
                                  white_area_center, height=0.03):
        """Pixel -> world XYZ for picking. The projection_matrix return is
        used elsewhere for `calculate_pixel_length` only.

        Round 14 CC.2: when a depth measurement + fitted table plane +
        endpoint pose are all available, route the world math through
        vendor `utils.calculate_world_position` (utils.py L294-343) so
        v5 gripping lands at the exact spot the vendor shape_recognition
        / object_sorting pick path would. This is the change that makes
        the arm grip the cube under it instead of off to the side.

        Fallback (no depth / no plane / no endpoint / vendor utils not
        importable) keeps the legacy pixels_to_world path that's worked
        since Round 10. _apply_world_offsets runs at the end either way."""
        # Always build projection_matrix for the caller (used by
        # calculate_pixel_length downstream).
        projection_matrix = np.row_stack(
            (np.column_stack((extristric[1], extristric[0])),
             np.array([[0, 0, 0, 1]])))
        px, py = int(position[0]), int(position[1])
        # Vendor path: only when EVERY input vendor needs is available.
        # depth here is in METRES (we hold it that way); vendor function
        # wants MM (it divides by 1000 internally). Convert just at the
        # call.
        used_vendor = False
        try:
            if (_VENDOR_UTILS_OK
                    and vendor_utils is not None
                    and self.table_plane is not None
                    and self._depth_available
                    and self._latest_depth is not None
                    # v5.1 FIX: only trust depth for the world position when it
                    # is colour-aligned to the RGB frame. Raw/unaligned depth
                    # samples the wrong pixel and corrupts X/Y/Z (X~0.30m, neg
                    # Z); fall back to the legacy AprilTag path, which is correct
                    # for this fixed camera.
                    and getattr(self, '_depth_is_aligned', False)):
                z_at = self._depth_at(px, py)
                if z_at is not None:
                    # _depth_at returns height-above-plane when plane is
                    # set (Round 12 Y3). Vendor calculate_world_position
                    # wants the RAW depth (mm) to do its own plane math.
                    # Re-sample the raw median depth here.
                    half = max(1, int(self.p('depth_window_px')))
                    img = self._latest_depth
                    h, w = img.shape[:2]
                    if 0 <= px < w and 0 <= py < h:
                        x0 = max(0, px - half); x1 = min(w, px + half + 1)
                        y0 = max(0, py - half); y1 = min(h, py + half + 1)
                        patch = img[y0:y1, x0:x1].ravel()
                        # v5.1 FIX (A3): gate on the clamped window size, not
                        # the full (2*half+1)^2 (see _depth_at for rationale).
                        n_total = patch.size
                        patch = patch[patch > 0]
                        if n_total > 0 and len(patch) >= n_total * 0.5:
                            depth_mm = float(np.median(patch))
                            # Round 15: endpoint pose taken from cached motion
                            # state, else identity (same conservative behavior
                            # the deleted _current_endpoint_pose used).
                            endpoint = getattr(self.motion, 'last_endpoint_pose',
                                               None)
                            if endpoint is None:
                                endpoint = np.eye(4, dtype=np.float64)
                            else:
                                endpoint = np.asarray(endpoint, dtype=np.float64)
                            K = np.asarray(self.intrinsic,
                                           dtype=np.float64).flatten()
                            world = vendor_utils.calculate_world_position(
                                px, py, depth_mm,
                                tuple(self.table_plane),
                                endpoint,
                                self.hand2cam_tf_matrix,
                                K)
                            position = np.array(world, dtype=np.float64)
                            used_vendor = True
        except Exception as e:
            _stage('pick', 'vendor calculate_world_position failed; '
                            'using legacy path', exc=e)

        if not used_vendor:
            # Legacy path (pre-Round 14). Identical to vendor's
            # object_sorting.py for the no-depth case.
            world_pose = common.pixels_to_world(
                [position], intrinsic, projection_matrix)[0]
            world_pose[0] = -world_pose[0]
            world_pose[1] = -world_pose[1]
            position = white_area_center[:3, 3] + world_pose
            position[2] = height
        position = self._apply_world_offsets(position)
        # v5.1: record whether the depth/vendor path set Z (object height above
        # the table, varies per object) or the legacy table-height fallback was
        # used, so the pick log can show which - and you can confirm depth-Z is
        # live and varying.
        self._last_worldpos_depth = used_vendor
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
        return self._select_gripper_yaw(target, target_info, gripper_size, yaw)

    def _select_gripper_yaw(self, target_points, points, gripper_size, yaw):
        """v5 fork of calculate_grasp_yaw.calculate_gripper_yaw_angle that
        prefers the orientation putting jaws across the SHORT OBB axis (more
        stable grip on elongated objects), tiebreaking by min |yaw|. Falls
        back to vendor min-yaw behaviour for square objects and when the
        `grasp_prefer_short_axis` toggle is off. Collision detection is
        unchanged - reuses the vendor `detect_collision` helper."""
        center_x, center_y = target_points[2][0], target_points[2][1]
        # v5.1 (B4): apply the hardware gripper-yaw correction to the OBB angle.
        try:
            angle = target_points[-1] + int(self.p('grasp_yaw_offset_deg'))
        except Exception:
            angle = target_points[-1]
        w, h = target_points[3]
        gripper_width, gripper_height = gripper_size
        half_w = gripper_width // 2
        half_h = gripper_height // 2
        collision_radius = math.sqrt(half_w ** 2 + half_h ** 2)

        angle1 = angle
        angle2 = angle - 90
        rect1 = ((center_x, center_y), (gripper_width, gripper_height), angle1)
        rect2 = ((center_x, center_y), (gripper_width, gripper_height), angle2)
        collisions = calculate_grasp_yaw.detect_collision(
            target_points, points, rect1, rect2, collision_radius)
        line1 = calculate_grasp_yaw.get_parallel_line(rect1)
        line2 = calculate_grasp_yaw.get_parallel_line(rect2)

        yaw1 = yaw + angle1
        yaw2 = yaw1 + 90 if yaw1 < 0 else yaw1 - 90

        # Short-axis preference: long axis at `angle` if w > h, so rect1
        # (oriented at `angle`) has gripper open direction along the long
        # axis = jaws clamp on the short axis. Mirror for h > w.
        prefer = None
        if bool(self.p('grasp_prefer_short_axis')) and max(w, h) > 0:
            ratio = abs(w - h) / max(w, h)
            if ratio >= float(self.p('grasp_short_axis_min_ratio')):
                prefer = 'yaw1' if w > h else 'yaw2'

        if len(collisions) == 0:
            if prefer == 'yaw1':
                return yaw1, line1, rect1
            if prefer == 'yaw2':
                return yaw2, line2, rect2
            return ((yaw1, line1, rect1) if abs(yaw1) < abs(yaw2)
                    else (yaw2, line2, rect2))
        elif len(collisions) == 1:
            if collisions[0] == 1:
                return yaw2, line2, rect2
            return yaw1, line1, rect1
        return None

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
        config_data = self._calibration_cfg()
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
        # v5.1 (depth-accurate grasp height): descend to the object-top Z
        # (position[2] - which on the depth/vendor path is the measured height
        # above the table, so it varies per object) minus grab_depth, but CLAMP
        # to a safe floor (min_descend_z_m) so a noisy/wrong depth reading can
        # never drive the gripper into the table. Logs the object-top Z, the
        # grab_depth and the final descend Z so you can verify depth-Z is live
        # (and tune grab_depth per class - e.g. the thin scaff - via the Places
        # tab / target_overrides).
        descend_z = position[2] - grab_depth
        min_dz = float(self.p('min_descend_z_m'))
        clamped = descend_z < min_dz
        if clamped:
            descend_z = min_dz
        _stage('pick', 'descend obj_top_z=%.3f grab_depth=%.3f -> z=%.3f%s '
                       '(worldpos=%s)' % (
                           position[2], grab_depth, descend_z,
                           ' [CLAMPED to floor]' if clamped else '',
                           'depth' if getattr(self, '_last_worldpos_depth', False)
                           else 'legacy/table'))
        descend = [position[0], position[1], descend_z]
        if self.motion.goto_pose(descend, pitch,
                                 duration=max(0.35, 0.7 * speed / aggression),
                                 parallel_base=False,
                                 pitch_range=(-90.0, 90.0)) is None:
            _stage('pick', 'descend IK failed')
            self.motion.set_gripper(open_pulse, 0.2 * speed)
            return False
        # Close: STANDARD fixed-pulse grasp by default; opt-in BETA
        # compliance "close until contact" with a per-class strength cap.
        outcome = 'standard'
        grip_t0 = time.time()
        if use_compliance:
            outcome = self.motion.compliance_grasp(
                target_pulse=max_strength, open_pulse=open_pulse,
                step=grasp_step, dwell=grasp_dwell,
                stall_thresh=grasp_stall, timeout=grasp_timeout,
                max_temp=grasp_max_temp)
            _stage('pick', f'compliance_grasp -> {outcome}')
        else:
            self.motion.set_gripper(close_pulse, close_dur)
        # Telemetry for the heartbeat mirror. Read final state once; cheap.
        final_pulse = None
        peak_temp = None
        try:
            st = self.motion.get_servo_state(GRIPPER_ID,
                                             fields=('position', 'temperature'))
            if st.get('position') is not None:
                final_pulse = int(st['position'])
            if st.get('temperature') is not None:
                peak_temp = int(st['temperature'])
        except Exception:
            pass
        self._last_grip = {
            'ts': time.time(),
            'label': label,
            'outcome': outcome,
            'final_pulse': final_pulse,
            'peak_temp': peak_temp,
            'duration_ms': int(1000 * (time.time() - grip_t0)),
            'source': 'pick',
        }
        # CLOSED-LOOP: if the compliance grip detected no contact (jaws
        # closed all the way without stalling) the object isn't between
        # the jaws - DON'T lift+place an empty hand. Bail so transport
        # opens the gripper, goes home, and the next sorting iteration
        # re-locks naturally. 'overheat' / 'aborted' also bail.
        # v5.1 FIX (A2): 'no_feedback' (compliance on, servo readback failed ->
        # one blind close, contact unconfirmed) now ALSO bails by default, so
        # we never lift+place an empty gripper. If this arm's gripper servo
        # genuinely can't report position and every grasp returns no_feedback,
        # set grasp_fail_on_no_feedback:false to restore the optimistic lift.
        # The 'standard' path (compliance disabled) trusts the fixed close.
        if outcome in ('closed', 'overheat', 'aborted') or (
                outcome == 'no_feedback'
                and bool(self.p('grasp_fail_on_no_feedback'))):
            _stage('pick', f'{outcome} - no confirmed object in jaws, bailing pick')
            self.motion.set_gripper(open_pulse, 0.2 * speed)
            return False
        # Settle so the servo holding torque has time to bite before lift.
        self.motion._sleep(settle)
        if self.motion.goto_pose(hover, pitch,
                                 duration=max(0.35, 0.7 * speed / aggression),
                                 parallel_base=False) is None:
            _stage('pick', 'lift IK failed')
            return False
        # v5.1 FIX (A4): retreat to a fixed safe transit pose (wrist centred)
        # BEFORE _do_place traverses horizontally to the drop zone. v5 lifted
        # only ~hover_height above the pick point, so the grasped object then
        # crossed the workspace at that low height -> collision risk with other
        # objects/walls. Mirrors the vendor pick() retreat to [0.11,0,0.09].
        # Best-effort: the object is already held, so a transit IK miss must
        # NOT drop it - we log and continue rather than returning False.
        if bool(self.p('safe_transit_enabled')) and not self.motion.aborted:
            self.motion.set_wrist(500, 0.3 * speed)
            if self.motion.goto_pose([0.11, 0.0, 0.09], 73,
                                     duration=max(0.4, 0.8 * speed / aggression),
                                     parallel_base=parallel_base) is None:
                _stage('pick', 'safe-transit IK miss (object held, continuing)')
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
                # len >= 3: a taught bin stores [x, y, z, 'taught']; take XYZ.
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    return [float(pos[0]), float(pos[1]), float(pos[2])]
                _stage('place', f'place_positions[{label}] malformed: {pos!r}')
        except Exception as e:
            _stage('place', 'place_positions parse failed', exc=e)
        return copy.deepcopy(self.place_position.get(label))

    def _is_taught_place(self, label):
        """True if place_positions[label] is a hand-taught coordinate (tagged
        with a 4th 'taught' element). Taught bins are real, physically-reached
        arm-base coordinates, so _do_place drives straight to them and SKIPS the
        kinematics scale/offset affine (that affine is a VISION-reach correction
        for camera-derived picks; applying it to a taught bin shifts the drop
        ~5% + a few mm off the saved coordinate)."""
        try:
            user = json.loads(self.p('place_positions') or '{}')
            pos = user.get(label) if isinstance(user, dict) else None
            return (isinstance(pos, (list, tuple)) and len(pos) >= 4
                    and str(pos[3]) == 'taught')
        except Exception:
            return False

    def _do_place(self, label):
        speed = max(0.1, 1.0 / float(self.p('motion_speed')))
        aggression = float(self.p('aggression'))
        position = self._resolve_place_position(label)
        if position is None:
            _stage('place', f'no place position configured for {label!r} '
                            f'- set one in the tuner UI Places tab')
            return False
        yaw = self.calculate_place_grasp_yaw(position, 0)
        # v5.1 bin-teach: a hand-taught bin is already a physically-reached
        # arm-base coordinate, so drive straight to it. The kinematics
        # scale/offset is a VISION-reach correction (applied to camera-derived
        # picks in transport_thread ~_apply_kinematics_calibration); applying it
        # to a taught bin re-corrects an already-correct point and lands ~5% +
        # offset off the saved coordinate. Keep it for legacy/default zones.
        if not self._is_taught_place(label):
            config_data = self._calibration_cfg()
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
                # v5.1 (thread-safety): take the handoff under _transport_lock,
                # then snapshot + COPY the position so the in-place
                # `position[2] += 0.01` below can't mutate a list the detection
                # thread still references (transport_info[0] may alias
                # detection_history). The lock is released the instant the
                # snapshot is taken - the long pick/place runs WITHOUT it - so
                # it only serialises the brief handoff, never motion/IO.
                with self._transport_lock:
                    _info = self.transport_info
                    if _info is None:
                        self.start_transport = False
                        self.target = None
                if _info is None:
                    continue
                position, yaw, target = list(_info[0]), _info[1], _info[2]
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
            with self._transport_lock:
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
                        # Round 17 NN.1: lock by LABEL only. Vendor's LAB-blob
                        # detection produces stable area-sorted indices so
                        # checking target[1] there is safe; v5 uses YOLO +
                        # position_reorder which can shuffle indices, breaking
                        # the lock every frame. When the locked target's
                        # label appears in this frame, prefer the candidate
                        # whose pixel position is closest to last_target_pixel.
                        if self.target is not None:
                            same_label = [t for t in target_info
                                          if t[0] == self.target[0]
                                          and self.target_labels.get(t[0], False)]
                            if same_label:
                                lp = getattr(self, '_last_target_pixel', None)
                                if lp is not None:
                                    same_label.sort(
                                        key=lambda t: (t[2][0]-lp[0])**2
                                                      + (t[2][1]-lp[1])**2)
                                # Reorder target_info so the chosen instance
                                # is iterated first; falls through to the
                                # standard lock branch below.
                                chosen = same_label[0]
                                target_info = [chosen] + [t for t in target_info
                                                          if t is not chosen]
                            else:
                                # Label disappeared - release lock cleanly so
                                # we can re-acquire next time it shows up.
                                self.target = None
                                self.count_still = 0
                                self.count_move = 0
                        for t in target_info:
                            # v5.1 FIX (multi-instance lock): track whether THIS
                            # iteration is the already-locked instance, so we can
                            # stop right after it (see the break below).
                            locked_this_t = False
                            if not self.target_labels.get(t[0], False):
                                continue
                            if self.target is not None:
                                # Label-only match (Round 17 NN.1); ignore
                                # the instance index t[1] which YOLO/position_
                                # reorder may have shuffled.
                                if self.target[0] != t[0]:
                                    continue
                                target_miss = False
                                self.target = t
                                locked_this_t = True
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
                                self._last_target_pixel = (t[2][0], t[2][1])
                                # Round 17 NN.4: one-shot LOCKED log so the
                                # session log captures the moment a target is
                                # first acquired.
                                _stage('sorting-loop',
                                       f'LOCKED on {t[0]} pixel={t[2]} '
                                       f'world=({position[0]:.3f},'
                                       f'{position[1]:.3f},{position[2]:.3f})')
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
                                # Round 17 NN.4: throttled diagnostics so the
                                # pushed logs show exactly why a pick does or
                                # does not fire.
                                if ticks % 30 == 0:
                                    _stage('sorting-loop',
                                           f'TRACK {t[0]} e_dist={e_distance:.4f} '
                                           f'(thresh={lock_thresh:.3f}) '
                                           f'still={self.count_still}/{still_thresh} '
                                           f'move={self.count_move}/{move_thresh}')
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
                                    # Round 17 NN.4: one-shot FIRING TRANSPORT
                                    # log proves the trigger landed.
                                    _stage('sorting-loop',
                                           f'FIRING TRANSPORT label={t[0]} pos='
                                           f'({avg_pos[0]:.3f},{avg_pos[1]:.3f},{avg_pos[2]:.3f}) '
                                           f'yaw={yaw_pulse}')
                                    with self._transport_lock:
                                        self.transport_info = [avg_pos, yaw_pulse, t]
                                        self.target = t
                                        self.start_transport = True
                            self.last_position = position
                            # Round 17 NN.1: remember the locked target's
                            # pixel so the next frame's label-only match can
                            # pick the closest instance.
                            self._last_target_pixel = (t[2][0], t[2][1])
                            # v5.1 FIX (multi-instance lock): once we've handled
                            # the locked instance (reordered to be first = the one
                            # nearest last frame's pixel), STOP. Previously the
                            # loop ran on to a SECOND same-label instance (e.g. the
                            # other red cube), overwrote self.target + last_position
                            # with it, and the ~0.15 m jump reset count_still every
                            # frame -> the still threshold was never reached and it
                            # never fired. Track exactly ONE instance per frame.
                            if locked_this_t:
                                break
                        if target_miss:
                            self.target_miss_count += 1
                        if self.target_miss_count > 10:
                            self.target_miss_count = 0
                            self.target = None
                            self._last_target_pixel = None
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

    def _depth_cam_info_callback(self, msg):
        """Cache depth-camera intrinsics so the depth plane refit's plane
        fit can build SearchPlane with correct fx/fy/cx/cy."""
        self._depth_cam_info = msg

    def _lookup_depth_color_tf(self):
        """Round 12 Y1: look up the static depth-to-color extrinsic so
        depth pixels can be aligned to the color frame even when the
        aligned topic isn't being published. Mirrors vendor
        calibration.py L77-101.

        Runs once at startup in a background thread. Falls back gracefully
        if the TF isn't published within 10 s (sets _depth_tf_ok=False)."""
        try:
            try:
                import tf2_ros
                from rclpy.duration import Duration
            except Exception as e:
                _stage('init', 'tf2_ros unavailable; depth alignment '
                               'falls back to topic-only', exc=e)
                return
            buf = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buf, self)
            self._tf_listener = listener  # keep reference alive
            deadline = time.time() + 10.0
            while time.time() < deadline:
                try:
                    tr = buf.lookup_transform(
                        'depth_cam_color_frame', 'depth_cam_link',
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.5))
                    M = common.xyz_quat_to_mat(
                        [tr.transform.translation.x,
                         tr.transform.translation.y,
                         tr.transform.translation.z],
                        [tr.transform.rotation.w,
                         tr.transform.rotation.x,
                         tr.transform.rotation.y,
                         tr.transform.rotation.z])
                    self._depth_to_color_tf = np.asarray(M, dtype=np.float64)
                    self._depth_tf_ok = True
                    # Round 14 CC.3: vendor calibration.py L99-100 composes
                    # the depth->color static TF into hand2cam so the
                    # world-position math sees the refined extrinsic.
                    # Mirror exactly: M @ default_hand2cam.
                    self.hand2cam_tf_matrix = np.matmul(
                        self._depth_to_color_tf, self.hand2cam_tf_matrix)
                    _stage('init', f'depth->color TF resolved + composed '
                                    f'into hand2cam')
                    return
                except Exception:
                    time.sleep(0.5)
            _stage('init', 'depth->color TF not resolved in 10s; depth '
                           'alignment falls back to depth_to_color topic')
        except Exception as e:
            _stage('init', 'depth TF lookup crashed', exc=e)

    def _depth_callback(self, msg, src_topic=''):
        """Cache the latest aligned depth frame for per-object Z lookup.
        Depth is uint16 in mm; converted to numpy array, kept as-is so
        _depth_at can index by colour pixel and convert per-window.

        v5.1 FIX (B3): topic selection happens here now (we subscribe to both
        candidates). The first topic to publish wins; the colour-aligned
        '/depth_cam/depth_to_color' stream always preempts the raw stream
        because _depth_at indexes depth by the colour-image pixel."""
        ALIGNED = '/depth_cam/depth_to_color'
        if src_topic and src_topic != self._depth_topic:
            if self._depth_topic == '' or src_topic == ALIGNED:
                self._depth_topic = src_topic
                self._depth_is_aligned = (src_topic == ALIGNED)
                _stage('camera', 'depth source = %s (colour-aligned=%s)'
                                 % (src_topic, src_topic == ALIGNED))
            else:
                # Frame from the non-selected (raw) topic while locked to the
                # aligned stream - ignore so we don't flip-flop.
                return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            if not getattr(self, '_depth_cb_warned', False):
                _stage('camera', 'depth cv_bridge failed (one-time warn)', exc=e)
                self._depth_cb_warned = True
            return
        self._latest_depth = img
        self._latest_depth_ts = time.time()
        self._depth_available = True
        # v5.1 FIX (P3): removed a dead per-frame compute. This block ran
        # vendor_utils.get_plane_values(full_frame, plane, K) on EVERY depth
        # frame and stored it in self._plane_values - but nothing anywhere ever
        # READ self._plane_values (only writes: the two `= None` inits and this
        # producer). A full-frame vectorised pass whose result was discarded.
        # _depth_at derives its own per-pixel plane height and the depth
        # heatmap computes its own, so this was pure waste; deleted.
        if not self._depth_first_logged:
            self._depth_first_logged = True
            _stage('camera', f'FIRST DEPTH FRAME on {self._depth_topic}: '
                             f'shape={img.shape} dtype={img.dtype}')

    def _depth_at(self, px, py, half=None):
        """Median depth (m) in a (2*half+1)^2 window around (px, py).
        Drops zeros (invalid pixels). Returns None if the depth subscription
        is dead, the latest frame is stale (>1s), or <50% of the window is
        valid.

        Round 12 Y3: when a table plane has been fit (self.table_plane),
        returns HEIGHT ABOVE PLANE (m). When no plane is available, returns
        absolute depth (m) - same as before for backward compatibility."""
        if not self._depth_available or self._latest_depth is None:
            return None
        if time.time() - self._latest_depth_ts > 1.0:
            return None
        if half is None:
            try:
                half = int(self.p('depth_window_px'))
            except Exception:
                half = 5
        img = self._latest_depth
        h, w = img.shape[:2]
        px = int(px); py = int(py)
        if px < 0 or py < 0 or px >= w or py >= h:
            return None
        x0 = max(0, px - half); x1 = min(w, px + half + 1)
        y0 = max(0, py - half); y1 = min(h, py + half + 1)
        patch = img[y0:y1, x0:x1].ravel()
        # v5.1 FIX (A3): validity threshold must be against the ACTUAL
        # (image-clamped) window, not the full (2*half+1)^2. Near any image
        # edge the patch is smaller, so the old full-area gate rejected every
        # detection within `half` px of a border even with perfect depth.
        n_total = patch.size
        patch = patch[patch > 0]
        if n_total == 0 or len(patch) < n_total * 0.5:
            return None
        z_m = float(np.median(patch)) / 1000.0  # mm -> m
        # Above-plane gating: if we have the plane equation, derive the
        # depth value AT the plane at this (u,v) and return the object's
        # height above it. Plane is in the depth-camera frame: the depth
        # value that would land on the plane along the camera ray.
        plane = self.table_plane
        info = self._depth_cam_info
        if plane is not None and info is not None:
            try:
                a, b, c, d = float(plane[0]), float(plane[1]), float(plane[2]), float(plane[3])
                kp = info.p if hasattr(info, 'p') and info.p else info.k
                if len(kp) >= 8:
                    fx = float(kp[0]); fy = float(kp[5])
                    cx_d = float(kp[2]); cy_d = float(kp[6])
                else:
                    fx = float(kp[0]); fy = float(kp[4])
                    cx_d = float(kp[2]); cy_d = float(kp[5])
                # Pixel ray (X, Y, Z) = (z*(u-cx)/fx, z*(v-cy)/fy, z).
                # Plane: aX+bY+cZ+d=0 -> z_plane = -d / (a*(u-cx)/fx +
                # b*(v-cy)/fy + c).
                dx = (px - cx_d) / fx
                dy = (py - cy_d) / fy
                denom = a * dx + b * dy + c
                if abs(denom) > 1e-9:
                    z_plane = -d / denom
                    return float(z_plane - z_m)  # height above plane
            except Exception:
                pass
        return z_m

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
        per sorting_loop iteration (at the detection rate, which can lag the
        camera) but the composited frame is published at full 30Hz, so the live
        camera background updates fluidly even when detection lags."""
        if overlay is None:
            return
        for op in overlay.get('yolo_ops', ()):
            kind = op[0]
            if kind == 'rect':
                cv2.rectangle(bgr, op[1], op[2], (0, 0, 255), 2)
            elif kind == 'circle':
                cv2.circle(bgr, op[1], 8, (0, 0, 255), -1)
            elif kind == 'obb':
                pts = np.array(op[1], dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(bgr, [pts], isClosed=True,
                              color=(0, 0, 255), thickness=2)
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

    def _draw_calibration_overlay(self, bgr):
        """Draw the workspace-calibration overlay: ROI rectangle, world axes,
        and a centre crosshair showing how the manual offset + scale move the
        arm's belief vs the true workspace centre.

        Drawn on top of any detection overlay. Active when
        calibrate_overlay_mode is 'manual' (persistent) or 'flash' (one-shot
        from _run_vendor_calibration; cleared by a worker thread after
        calibrate_flash_secs)."""
        mode = self.p('calibrate_overlay_mode')
        if mode == 'off':
            return
        # Status line top-left so the user can read live values.
        gx = float(self.p('grip_offset_x'))
        gy = float(self.p('grip_offset_y'))
        ws = float(self.p('workspace_scale'))
        header = ('MANUAL CALIBRATE' if mode == 'manual' else
                  'CALIBRATED (auto)')
        cv2.putText(bgr,
                    f'{header}  X={gx:+.3f}m  Y={gy:+.3f}m  scale={ws:.3f}',
                    (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 2)
        # Need camera intrinsics + extrinsics + workspace centre to project.
        if (self.intrinsic is None or self.distortion is None
                or not hasattr(self, 'extristric') or self.extristric is None
                or self.white_area_center is None):
            cv2.putText(bgr, '(awaiting calibration - press CALIBRATE)',
                        (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 200, 255), 1)
            return
        try:
            tvec, rmat = self.extristric
            centre_w = np.array(self.white_area_center[:3, 3],
                                dtype=np.float64)
            # Project workspace centre + axis tips. World axes drawn 5 cm
            # in +X (red) and +Y (green) from the centre, lifted to the
            # picking plane (z=0.03).
            pts_w = np.array([
                centre_w,
                centre_w + np.array([0.05, 0.0, 0.0]),
                centre_w + np.array([0.0, 0.05, 0.0]),
            ], dtype=np.float64)
            imgpts, _ = cv2.projectPoints(pts_w, np.array(rmat),
                                          np.array(tvec),
                                          self.intrinsic, self.distortion)
            imgpts = np.int32(imgpts).reshape(-1, 2)
            cx, cy = int(imgpts[0][0]), int(imgpts[0][1])
            xtip = (int(imgpts[1][0]), int(imgpts[1][1]))
            ytip = (int(imgpts[2][0]), int(imgpts[2][1]))
            # True workspace centre (cyan +).
            cv2.drawMarker(bgr, (cx, cy), (255, 255, 0),
                           cv2.MARKER_CROSS, 18, 2)
            cv2.putText(bgr, '0,0', (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            # World axes from centre.
            cv2.arrowedLine(bgr, (cx, cy), xtip, (0, 0, 255), 2,
                            tipLength=0.2)
            cv2.putText(bgr, 'X', (xtip[0] + 4, xtip[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.arrowedLine(bgr, (cx, cy), ytip, (0, 255, 0), 2,
                            tipLength=0.2)
            cv2.putText(bgr, 'Y', (ytip[0] + 4, ytip[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Workspace rectangle sized by workspace_size_x/y. Round 12 W1:
            # build corners by FULL-POSE composition with white_area_center
            # (vendor draw_retangle pattern - calibration.py L322-326), so
            # corners rotate with the workspace plane instead of being
            # axis-aligned in the camera frame. The previous version added
            # axis-aligned offsets to white_area_center[:3,3] only, which
            # ignored the workspace pose rotation and (when a corner fell
            # behind the camera) collapsed to a triangle via int32(NaN).
            sx = float(self.p('workspace_size_x')) / 2.0
            sy = float(self.p('workspace_size_y')) / 2.0
            W = np.asarray(self.white_area_center, dtype=np.float64)
            # Vendor offset convention (calibration.py L322-326):
            # local (height/2, width/2, 0) maps to sy, sx, 0.
            local_offsets = [(+sy, +sx, 0.0), (-sy, +sx, 0.0),
                             (-sy, -sx, 0.0), (+sy, -sx, 0.0)]
            corners_w = []
            for lx, ly, lz in local_offsets:
                M_off = common.xyz_euler_to_mat((lx, ly, lz), (0, 0, 0))
                P = np.matmul(W, M_off)
                corners_w.append(P[:3, 3])
            corners_w = np.array(corners_w, dtype=np.float64)
            cimg, _ = cv2.projectPoints(corners_w, np.array(rmat),
                                        np.array(tvec),
                                        self.intrinsic, self.distortion)
            pts = cimg.reshape(-1, 2)
            # NaN / Inf / off-screen guard. Corners behind the camera
            # project to NaN; without this they cast to int32 = INT_MIN
            # and the polyline draws a triangle (Round 11 visible bug).
            H_img, W_img = bgr.shape[:2]
            finite = np.isfinite(pts).all(axis=1)
            in_window = ((pts[:, 0] > -W_img) & (pts[:, 0] < 2 * W_img)
                         & (pts[:, 1] > -H_img) & (pts[:, 1] < 2 * H_img))
            ok = finite & in_window
            if int(ok.sum()) == 4:
                poly = np.int32(pts).reshape(-1, 1, 2)
                cv2.polylines(bgr, [poly], True, (0, 255, 255), 2)
            elif int(ok.sum()) >= 2:
                idx = np.where(ok)[0]
                for a, b in zip(idx[:-1], idx[1:]):
                    cv2.line(bgr,
                             (int(pts[a, 0]), int(pts[a, 1])),
                             (int(pts[b, 0]), int(pts[b, 1])),
                             (0, 255, 255), 2)
                cv2.putText(bgr, '(workspace partly off-screen)',
                            (8, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 200, 255), 1)
            else:
                cv2.putText(bgr,
                            '(workspace corners off-screen - recalibrate)',
                            (8, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 200, 255), 2)
            # Round 12 Y5: optional depth heatmap inside the workspace
            # rectangle so the user can visually confirm depth is reading
            # the objects and is aligned with RGB.
            if (bool(self.p('overlay_depth_view'))
                    and self._latest_depth is not None
                    and self._depth_available):
                try:
                    depth_m = self._latest_depth.astype(np.float32) / 1000.0
                    dmin = float(self.p('depth_min_z_m'))
                    dmax = float(self.p('depth_max_z_m'))
                    # Plane-relative heatmap when plane is set: small range
                    # around the plane height. Else: absolute depth band.
                    if self.table_plane is not None:
                        vmin = -0.05; vmax = max(0.10, dmax)
                    else:
                        vmin = max(0.10, dmin); vmax = max(0.20, dmax)
                    clip = np.clip(depth_m, vmin, vmax)
                    norm = ((clip - vmin) / max(1e-3, (vmax - vmin)) * 255
                            ).astype(np.uint8)
                    cmap = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                    # Resize to color frame if shapes differ.
                    if cmap.shape[:2] != bgr.shape[:2]:
                        cmap = cv2.resize(cmap,
                                          (bgr.shape[1], bgr.shape[0]),
                                          interpolation=cv2.INTER_NEAREST)
                    # Mask to workspace polygon (whichever corners projected
                    # successfully above).
                    if int(ok.sum()) >= 3:
                        mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
                        cv2.fillPoly(mask,
                                     [np.int32(pts[ok]).reshape(-1, 1, 2)],
                                     255)
                        blended = cv2.addWeighted(bgr, 0.55, cmap, 0.45, 0)
                        bgr[mask > 0] = blended[mask > 0]
                except Exception as e:
                    _stage('overlay', 'depth heatmap blend failed', exc=e)
            # Legend for the per-object + bin markers below.
            cv2.putText(bgr, 'yellow = arm grab aim   magenta = place bin',
                        (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 200, 255), 1)
        except Exception:
            # Overlay must never break the publisher; swallow projection
            # failures silently and skip the markers.
            pass

        # Per-object pick-aim markers: for each detected object, run the FULL
        # pick math (offsets + scale) and project the resulting world target
        # back onto the image. Marker sits on the object when calibration is
        # right; the line to the detected centre is the residual being dialed
        # out. Recomputed each tick from live params, so dragging the sliders
        # (incl. workspace_scale) moves every marker in real time.
        overlay = self._latest_overlay
        targets = (overlay or {}).get('targets', ()) if overlay else ()
        for t in targets:
            try:
                px, py = int(t[2][0]), int(t[2][1])
                # Use the measured depth as the projection height when
                # available so the aim marker tracks the real top of the
                # object (matches what the pick will actually do).
                z_meas = self._depth_at(px, py)
                height = float(z_meas) if z_meas is not None else 0.03
                world, _ = self.get_object_world_position(
                    (px, py), self.intrinsic, self.extristric,
                    self.white_area_center, height=height)
                aim = self._project_world_to_pixel(world)
                if aim is None:
                    continue   # behind camera; skip the marker
                ax, ay = aim
                cv2.line(bgr, (px, py), (ax, ay), (0, 200, 255), 1)
                cv2.drawMarker(bgr, (ax, ay), (0, 200, 255),
                               cv2.MARKER_TILTED_CROSS, 16, 2)
                if z_meas is not None:
                    cv2.putText(bgr, f'{int(z_meas * 1000)}mm',
                                (ax + 10, ay + 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (0, 200, 255), 1)
            except Exception:
                continue

        # Place-bin markers: where each class gets dropped on the real table.
        try:
            bins = dict(self.DEFAULT_PLACE_POSITIONS)
            user_pp = json.loads(self.p('place_positions') or '{}')
            if isinstance(user_pp, dict):
                for k, v in user_pp.items():
                    # len >= 3: taught bins store [x, y, z, 'taught']; take XYZ.
                    if isinstance(v, (list, tuple)) and len(v) >= 3:
                        bins[k] = [float(v[0]), float(v[1]), float(v[2])]
            labels = getattr(self, 'target_labels', None) or {}
            for label, xyz in bins.items():
                if labels and label not in labels:
                    continue
                proj = self._project_world_to_pixel(list(xyz))
                if proj is None:
                    continue
                bx, by = proj
                cv2.drawMarker(bgr, (bx, by), (255, 0, 255),
                               cv2.MARKER_DIAMOND, 14, 2)
                cv2.putText(bgr, f'{label} bin', (bx + 6, by - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        except Exception:
            pass

    def _project_world_to_pixel(self, world_xyz):
        """Project a position from the get_object_world_position output frame
        back to a pixel for the overlay.

        Round 12 W0: route through the vendor inverse transform. The chain
        in get_object_world_position is:
            position = white_area_center[:3,3] + (-wp_x, -wp_y, wp_z)
        followed by grip_offset_x/y + workspace_scale. To reverse:
        subtract white_area_center, undo the X/Y sign-flip, then use
        cv2.projectPoints (the exact inverse of common.pixels_to_world).
        The user offsets stay APPLIED on purpose - the overlay marker is
        meant to SHOW where the offset moved the aim point.

        At grip_offset_x/y = 0 and workspace_scale = 1, the round-trip
        property holds: project(pixel -> world) sits exactly on `pixel`.

        NaN/Inf guard: projectPoints returns NaN for points behind the
        camera. Caller handles None by skipping the marker."""
        p = np.array(world_xyz, dtype=np.float64).copy()
        p = p - self.white_area_center[:3, 3]
        p[0] = -p[0]
        p[1] = -p[1]
        tvec, rmat = self.extristric
        img, _ = cv2.projectPoints(np.array([p]), np.array(rmat),
                                   np.array(tvec), self.intrinsic,
                                   self.distortion)
        out = img.reshape(-1, 2)[0]
        if not (np.isfinite(out[0]) and np.isfinite(out[1])):
            return None
        return int(out[0]), int(out[1])

    def _raw_republish_tick(self):
        """30 Hz publisher: always grabs the latest raw camera frame and
        composites the latest detection overlay onto it. The live background
        updates at camera rate even when the detection loop is slow; overlays
        may lag by 0.5-2s but the user can see the robot moving in real time.
        """
        bgr_src = self._latest_raw_bgr
        if bgr_src is None:
            return
        # v5.1 FIX (P1): skip the whole overlay-composite + cv_bridge + DDS
        # publish when nobody is watching. The raw result_publisher ran this at
        # 30 Hz even with zero subscribers (the JPEG sibling is already gated in
        # _publish_image). Bail when neither the raw nor the compressed result
        # topic has a subscriber.
        try:
            watchers = self.result_publisher.get_subscription_count()
            cpub = getattr(self, 'compressed_publisher', None)
            if cpub is not None:
                watchers += cpub.get_subscription_count()
            if watchers == 0:
                return
        except Exception:
            pass
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
            # refreshing while the camera keeps moving. 5s tolerates a slow
            # detection iteration without dropping the overlay.
            if age <= 5.0:
                try:
                    self._draw_overlay(bgr, overlay)
                except Exception as e:
                    if not getattr(self, '_overlay_err_warned', False):
                        _stage('camera', 'overlay draw failed (one-time warn)', exc=e)
                        self._overlay_err_warned = True
        # Calibration overlay sits on top of the detection overlay; cheap
        # no-op when mode == 'off' (the default).
        try:
            self._draw_calibration_overlay(bgr)
        except Exception as e:
            if not getattr(self, '_cal_overlay_warned', False):
                _stage('camera', 'cal overlay draw failed (one-time warn)', exc=e)
                self._cal_overlay_warned = True
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
