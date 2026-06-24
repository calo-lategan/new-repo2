#!/usr/bin/env python3
# coding: utf8
# Live tuner UI for custom_sortingv5.
#
# Adds over v2's tune_ui:
#  - Engine hot-swap: file picker / dropdown of available .engine files.
#    Calls /custom_sortingv5/load_engine - the inference worker swaps
#    between frames, no restart needed.
#  - Profile manager: list, save, load named profiles from
#    ~/jetarm_v5_profiles/.
#  - "Save as default" button - persists the current settings as the
#    profile loaded on every startup.
#  - All v2 controls (Start / Stop / Calibrate, sliders, presets).
#  - Tabs reorganised: Control / Speed / Grip / Vision / Models / Profiles / Toggles.

import os
import sys
import json
import time
import argparse
import shutil
import subprocess
import threading
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path

# ---- Session log file (Round 12 commit Z) ----
# Mirror UI events + errors into the same session log directory the node
# writes to, so a single push_logs.sh run captures the full picture.
_UI_LOG_DIR = os.environ.get(
    'JETARM_V5_LOG_DIR',
    os.path.expanduser('~/jetarm_v5/logs'))
_UI_LOG_FILE = None
_UI_LOG_LOCK = threading.Lock()
_UI_TS = time.strftime('%Y-%m-%d_%H-%M-%S')


def _ui_log(tag, msg, exc=None):
    """Append a UI event to the session log. Also prints to stderr for the
    launching terminal."""
    line = f'[ui][{tag}] {msg}'
    print(line, file=sys.stderr, flush=True)
    global _UI_LOG_FILE
    if _UI_LOG_FILE is None:
        try:
            os.makedirs(_UI_LOG_DIR, exist_ok=True)
            path = os.path.join(_UI_LOG_DIR,
                                f'v5_ui_{os.getpid()}_{_UI_TS}.log')
            _UI_LOG_FILE = open(path, 'a', buffering=1)
            _UI_LOG_FILE.write(f'# v5 ui session {_UI_TS} pid={os.getpid()}\n')
        except Exception:
            _UI_LOG_FILE = None
    if _UI_LOG_FILE is None:
        return
    try:
        with _UI_LOG_LOCK:
            _UI_LOG_FILE.write(f'{time.strftime("%H:%M:%S")} {line}\n')
            if exc is not None:
                _UI_LOG_FILE.write(f'{time.strftime("%H:%M:%S")} '
                                   f'EXCEPTION: {type(exc).__name__}: {exc}\n')
                _UI_LOG_FILE.write(traceback.format_exc())
    except Exception:
        pass


def _ui_excepthook(exc_type, exc_value, tb):
    _ui_log('uncaught', f'{exc_type.__name__}: {exc_value}')
    if _UI_LOG_FILE is not None:
        try:
            with _UI_LOG_LOCK:
                _UI_LOG_FILE.write(''.join(
                    traceback.format_exception(exc_type, exc_value, tb)))
        except Exception:
            pass
    sys.__excepthook__(exc_type, exc_value, tb)


sys.excepthook = _ui_excepthook

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from interfaces.srv import SetStringBool


PROFILES_DIR = Path(os.environ.get('JETARM_V5_PROFILES',
                                   str(Path.home() / 'jetarm_v5_profiles')))
DEFAULT_ENGINES_DIR = Path(os.environ.get('JETARM_V5_ENGINES_DIR',
                                          '/home/ubuntu/third_party_ros2/data'))


# v5: live params (push on slider release) - everything EXCEPT the model
# knobs, which are buffered in the Model tab until SAVE.
FLOAT_PARAMS = [
    ('motion_speed',          0.3, 2.5, 0.05),
    ('aggression',            0.3, 2.0, 0.05),
    ('hover_height',          0.02, 0.15, 0.005),
    ('approach_dwell',        0.0, 1.0, 0.05),
    ('lock_distance_thresh',  0.001, 0.05, 0.001),
    ('gripper_close_duration', 0.1, 2.0, 0.05),
    ('gripper_settle',        0.0, 1.5, 0.05),
    ('grab_depth',            0.0, 0.05, 0.001),
    # Force-limited grasp BETA (close until contact). Live-editable.
    ('grasp_step_dwell',      0.03, 0.3, 0.01),
    ('grasp_timeout',         0.3, 5.0, 0.1),
    ('publish_max_hz',        0.0,  60.0, 1.0),
    ('publish_scale',         0.25, 1.0, 0.05),
]

INT_PARAMS = [
    ('count_still_threshold',    1, 30, 1),
    ('count_move_threshold',     1, 30, 1),
    ('detection_avg_frames',     1, 10, 1),
    ('gripper_open_pulse',       50, 500, 5),
    ('gripper_close_pulse',      300, 700, 5),
    # Force-limited grasp BETA tunables.
    ('grasp_step_pulse',         5, 80, 1),
    ('grasp_stall_pulse',        1, 40, 1),
    ('grasp_max_temp',           40, 80, 1),
    ('publish_jpeg_quality',     30, 95, 5),
]

# Friendly display names + help for otherwise-opaque param keys. The ROS
# param name stays the dict key; only the visible label changes.
PARAM_LABELS = {
    'grasp_step_pulse':  ('Grip step', 'pulses/step - smaller = gentler contact'),
    'grasp_step_dwell':  ('Grip step dwell', 'sec - wait per step for servo + readback'),
    'grasp_stall_pulse': ('Grip stall threshold', 'pulses - below this = contact detected'),
    'grasp_timeout':     ('Grip timeout', 'sec - failsafe budget'),
    'grasp_max_temp':    ('Grip max temp', 'degC - stop to protect the gripper servo'),
    'gripper_close_pulse': ('Gripper close pulse', 'standard-grasp close target'),
    'gripper_open_pulse':  ('Gripper open pulse', 'jaw-open position'),
    'gripper_settle':    ('Grip settle', 'sec dwell after close before lift'),
    'grab_depth':        ('Grab depth', 'm below detected z so jaws wrap the body'),
    'detection_offset_x': ('Overlay offset X', 'px - nudge boxes +right/-left onto objects'),
    'detection_offset_y': ('Overlay offset Y', 'px - nudge boxes +down/-up onto objects'),
    'grip_offset_x':     ('Offset X', 'm - shift the arm landing left/right'),
    'grip_offset_y':     ('Offset Y', 'm - shift the arm landing forward/back'),
    'workspace_scale':   ('Workspace scale (Z)', 'how big/small the world map is (1.0 = no change)'),
    'workspace_size_x':  ('Workspace size X', 'm - width of your physical mat'),
    'workspace_size_y':  ('Workspace size Y', 'm - depth of your physical mat'),
    'grasp_short_axis_min_ratio': ('Short-axis gate', '|w-h|/max(w,h) above which short-axis preference applies (cubes: stay vendor)'),
    'depth_window_px':   ('Depth window', 'half-size of median window around each detection (px)'),
    'depth_min_z_m':     ('Depth min Z', 'm - drop detections below this (reflections)'),
    'depth_max_z_m':     ('Depth max Z', 'm - drop detections above this (hand)'),
}

# v5 Round 2: live YOLO knobs rendered on the Detection tab (apply on release
# like every other slider). Persisted by the Detection tab's "Save & Apply".
MODEL_FLOAT_PARAMS = [
    ('yolo_conf_thresh',      0.05, 0.95, 0.01),
    ('yolo_iou_thresh',       0.10, 0.90, 0.01),
    ('inference_max_hz',      0.0,  60.0, 1.0),
    # Aspect-ratio gate for the short-axis grasp preference. Below this
    # the OBB is "near square" and vendor min-yaw is used.
    ('grasp_short_axis_min_ratio', 0.02, 0.50, 0.01),
]
MODEL_INT_PARAMS = [
    ('yolo_max_det',             1, 300, 1),
    # Pixel-space overlay nudge - shift boxes to sit on the objects.
    ('detection_offset_x',    -300, 300, 1),
    ('detection_offset_y',    -300, 300, 1),
]
# Round 12 Y6: ALL depth + camera + workspace controls live on the
# Calibrate tab. Detection tab keeps YOLO + pixel-offset only.
CALIB_DEPTH_FLOAT_PARAMS = [
    # Depth sanity-gate band. Once a plane is fit, the gate uses
    # height-above-plane and depth_min_z_m is largely unused.
    ('depth_min_z_m',        -0.10, 0.20, 0.005),
    ('depth_max_z_m',         0.05, 0.50, 0.005),
]
CALIB_DEPTH_INT_PARAMS = [
    # Half-size (in pixels) of the median window used for per-object depth.
    ('depth_window_px',          1, 30, 1),
]
# Calibrate tab: manual world-XY offset + workspace scale + workspace size.
# grip_offset_x/y were on Detection in commit Q; commit R moves them here
# (their proper home) alongside workspace_scale. Ranges widened in Round 11
# (±0.50 m, was ±0.10) for users compensating significant calibration drift
# without re-running the AprilTag. workspace_size_x/y added Round 11 so the
# user sizes the overlay rectangle to their physical mat.
CALIB_FLOAT_PARAMS = [
    ('grip_offset_x',        -0.50, 0.50, 0.005),
    ('grip_offset_y',        -0.50, 0.50, 0.005),
    ('workspace_scale',       0.50, 1.50, 0.01),
    ('workspace_size_x',      0.05, 1.00, 0.005),
    ('workspace_size_y',      0.05, 1.00, 0.005),
]
# Factory defaults for the Detection tab "Reset knobs to defaults" button.
# Keep in sync with the node's declared defaults in custom_sortingv5.py
# (_v5_tunables block around L615 - yolo_conf_thresh / yolo_iou_thresh /
# yolo_max_det / inference_max_hz).
MODEL_DEFAULTS = {
    'yolo_conf_thresh': 0.60,
    'yolo_iou_thresh':  0.45,
    'yolo_max_det':     5,
    'inference_max_hz': 15.0,
}

BOOL_PARAMS = [
    'parallel_base_motion',
    # Force-limited grasp BETA: OFF = standard close-to-pulse grasp.
    'compliance_grasp_enabled',
    # Grasp orientation: prefer clamping on the SHORT OBB axis vs vendor min-yaw.
    'grasp_prefer_short_axis',
    # Depth integration: use measured Z per object instead of fixed table plane.
    'use_depth_for_z',
    'inference_warmup',
    'hot_log_inference_ms',
    # Independent stop/start of camera subscription + inference.
    'enable_camera_sub',
    'enable_inference',
]

# v5.1: the three speed presets - SLOW / MEDIUM / FAST. Unlike the old live
# presets, v5.1 ships them as PARTIAL ros2-param YAMLs (profiles/slow.yaml,
# medium.yaml, fast.yaml) and the 3 buttons LOAD them through the node's
# existing, hardened load_profile service - which applies params one at a time
# with type/range tolerance - instead of pushing them live key-by-key (the old
# _apply_preset path silently dropped any type-mismatched key). Each YAML tweaks
# ONLY the motion, grip-timing and detection-gating knobs that trade speed vs
# accuracy; engine, calibration, depth and place-zone params are NOT touched.
# Medium == default.yaml. install.sh seeds the three files next to default.yaml.
# (button label, profile basename loaded via load_profile)
SPEED_PRESETS = [
    ('Slow', 'slow'),
    ('Medium', 'medium'),
    ('Fast', 'fast'),
]


def _slugify_preset(name):
    """Lowercase, spaces/ampersands -> hyphen, strip the .yaml suffix."""
    s = str(name or '').strip().lower()
    if s.endswith('.yaml'):
        s = s[:-5]
    out = []
    for ch in s:
        out.append(ch if (ch.isalnum() or ch == '-') else '-')
    slug = ''.join(out)
    while '--' in slug:
        slug = slug.replace('--', '-')
    return slug.strip('-')


# Names a custom preset may not take (would clobber a speed preset or boot file).
RESERVED_PRESET_SLUGS = (
    {slug for _, slug in SPEED_PRESETS} | {'default', 'yolo'}
)


class TunerClient(Node):
    def __init__(self, target_node, client_name=None):
        super().__init__(client_name or 'custom_sortingv5_tuner')
        self.target = target_node
        self.set_cli = self.create_client(SetParameters, f'/{target_node}/set_parameters')
        self.get_cli = self.create_client(GetParameters, f'/{target_node}/get_parameters')
        self.list_cli = self.create_client(ListParameters, f'/{target_node}/list_parameters')
        self.enable_cli = self.create_client(SetBool, f'/{target_node}/enable_sorting')
        self.recalibrate_cli = self.create_client(Trigger, f'/{target_node}/recalibrate')
        self.run_calibration_cli = self.create_client(Trigger, f'/{target_node}/run_calibration')
        # Round 12 Y6: refit just the depth-table plane without
        # re-running the AprilTag step.
        self.depth_plane_refit_cli = self.create_client(
            Trigger, f'/{target_node}/depth_plane_refit')
        # Round 14 AA: LAB color calibration clients (vendor lab_manager
        # service signatures).
        from interfaces.srv import (
            StashRange as _StashRange, GetRange as _GetRange,
            ChangeRange as _ChangeRange, GetAllColorName as _GetAllColorName,
        )
        # Round 15: drive the vendor lab_manager_node directly. v5 used to
        # host its own /custom_sortingv5/lab_* services (Round 14) but
        # those have been removed - the vendor node IS the canonical
        # implementation and lives at /lab_manager/* on the same domain.
        self.lab_enter_cli = self.create_client(
            Trigger, '/lab_manager/enter')
        self.lab_exit_cli = self.create_client(
            Trigger, '/lab_manager/exit')
        self.lab_save_to_disk_cli = self.create_client(
            Trigger, '/lab_manager/save_to_disk')
        self.lab_get_range_cli = self.create_client(
            _GetRange, '/lab_manager/get_range')
        self.lab_change_range_cli = self.create_client(
            _ChangeRange, '/lab_manager/change_range')
        self.lab_stash_range_cli = self.create_client(
            _StashRange, '/lab_manager/stash_range')
        self.lab_get_all_names_cli = self.create_client(
            _GetAllColorName, '/lab_manager/get_all_color_name')
        self._lab_StashRange = _StashRange
        self._lab_GetRange = _GetRange
        self._lab_ChangeRange = _ChangeRange
        self._lab_GetAllColorName = _GetAllColorName
        self.enter_cli = self.create_client(Trigger, f'/{target_node}/enter')
        self.exit_cli = self.create_client(Trigger, f'/{target_node}/exit')
        self.load_engine_cli = self.create_client(SetStringBool, f'/{target_node}/load_engine')
        self.save_profile_cli = self.create_client(SetStringBool, f'/{target_node}/save_profile')
        self.load_profile_cli = self.create_client(SetStringBool, f'/{target_node}/load_profile')
        self.save_default_cli = self.create_client(Trigger, f'/{target_node}/save_as_default')
        # v5: model-config buffered save.
        # v5 BETA: per-class grip-test orchestrator.
        self.test_grip_cli = self.create_client(
            SetStringBool, f'/{target_node}/test_grip')
        self.save_yolo_cli = self.create_client(SetStringBool,
                                                f'/{target_node}/save_yolo_config')
        # v5 Round 2: generic per-tab "Save & Apply" (apply + persist to default).
        self.apply_persist_cli = self.create_client(
            SetStringBool, f'/{target_node}/apply_and_persist')
        # v5: re-init the currently loaded engine in place (no path change).
        self.reload_engine_cli = self.create_client(
            Trigger, f'/{target_node}/reload_engine')
        # v5.1 bin-teach: jog the arm (joint-by-joint or world XYZ), read the
        # live world coordinate, GOTO/SAVE a bin coordinate per class, and
        # teach the workspace centre/edges. JSON command in data_str; feedback
        # comes back on the heartbeat (last_teach_msg).
        self.teach_cli = self.create_client(
            SetStringBool, f'/{target_node}/teach')
        # Live heartbeat mirror: the node publishes its 5s heartbeat as
        # JSON on ~/status. The background rclpy.spin thread services this
        # subscription; the UI polls latest_status via root.after().
        self.latest_status = None
        self.status_rx_t = 0.0
        self.status_sub = self.create_subscription(
            String, f'/{target_node}/status', self._on_status, 1)
        # v5.1 bin-teach: low-latency teach readout (world pose, servo pulses,
        # staged workspace centre/edges, last message). Published immediately
        # after each teach action so the Bin Teach tab doesn't wait for the 5s
        # heartbeat.
        self.latest_teach = None
        self.teach_rx_t = 0.0
        self.teach_status_sub = self.create_subscription(
            String, f'/{target_node}/teach_status', self._on_teach_status, 1)

    def _on_status(self, msg):
        # Runs on the background spin thread: only write plain attributes
        # here (thread-safe); all Tk updates happen in the UI's after() poll.
        try:
            self.latest_status = json.loads(msg.data)
            self.status_rx_t = time.time()
        except Exception:
            pass

    def _on_teach_status(self, msg):
        try:
            self.latest_teach = json.loads(msg.data)
            self.teach_rx_t = time.time()
        except Exception:
            pass

    def wait_ready(self, timeout=10.0):
        return (self.set_cli.wait_for_service(timeout_sec=timeout)
                and self.get_cli.wait_for_service(timeout_sec=timeout))

    def _wait_future(self, future, timeout):
        """Poll until the background rclpy.spin thread completes the future.

        Previously every call site used rclpy.spin_until_future_complete,
        which spins the node's executor from whatever thread called it -
        the Tk main thread (slider release), worker `go()` threads, AND the
        background rclpy.spin thread all at once. Concurrent spins on one
        node race: the background spinner consumes the response the
        foreground spinner is waiting on, and calls stall to their timeout
        ("NODE NOT REACHABLE" with a healthy node). Polling future.done()
        leaves all executor work to the single background spin thread.
        """
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.done()

    def get_values(self, names):
        req = GetParameters.Request(); req.names = list(names)
        future = self.get_cli.call_async(req)
        self._wait_future(future, 2.0)
        if not future.done() or future.result() is None:
            return {}
        out = {}
        for name, val in zip(names, future.result().values):
            if val.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = val.double_value
            elif val.type == ParameterType.PARAMETER_INTEGER:
                out[name] = val.integer_value
            elif val.type == ParameterType.PARAMETER_BOOL:
                out[name] = val.bool_value
            elif val.type == ParameterType.PARAMETER_STRING:
                out[name] = val.string_value
        return out

    def set_value(self, name, value):
        p = Parameter(); p.name = name
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL; pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER; pv.integer_value = int(value)
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE; pv.double_value = float(value)
        else:
            pv.type = ParameterType.PARAMETER_STRING; pv.string_value = str(value)
        p.value = pv
        req = SetParameters.Request(); req.parameters = [p]
        future = self.set_cli.call_async(req)
        # v5.1 FIX (A6a): return whether the node accepted the param so callers
        # (presets) can surface rejected keys instead of failing silently.
        res = self._wait_future(future, 2.0)
        try:
            return bool(res.results[0].successful)
        except Exception:
            return None

    def _trigger(self, client):
        # Round 17 OO.1: 5 s discovery wait. 1 s was too short on the Jetson
        # under load (service discovery can lag 2-4 s after a node starts).
        if not client.wait_for_service(timeout_sec=5.0): return False
        future = client.call_async(Trigger.Request())
        self._wait_future(future, 5.0)
        return future.done() and future.result() is not None and future.result().success

    def _trigger_with_msg(self, client, discovery_wait=False):
        """Round 17 OO.2/OO.3: like _trigger, but returns (success, message).
        When discovery_wait=False we skip wait_for_service entirely and let
        call_async queue the request - matches the working pattern of
        lab_get_range. Returns ('timeout', '<reason>') on failure."""
        if discovery_wait and not client.wait_for_service(timeout_sec=5.0):
            return False, 'service unreachable (discovery timeout)'
        future = client.call_async(Trigger.Request())
        self._wait_future(future, 5.0)
        if not future.done():
            return False, 'service call timeout'
        res = future.result()
        if res is None:
            return False, 'no response from service'
        return bool(res.success), getattr(res, 'message', '') or ''

    def _set_string_bool(self, client, s, b=True):
        if not client.wait_for_service(timeout_sec=5.0): return False
        req = SetStringBool.Request(); req.data_str = s; req.data_bool = b
        future = client.call_async(req)
        self._wait_future(future, 5.0)
        return future.done() and future.result() is not None and future.result().success

    def call_enable_sorting(self, enable):
        # v5.1 FIX (A6b): 1s discovery wait was too short on a loaded Jetson
        # (every other client uses 5s), so START could spuriously report the
        # node unreachable even though enter() had just succeeded.
        if not self.enable_cli.wait_for_service(timeout_sec=5.0): return False
        req = SetBool.Request(); req.data = bool(enable)
        future = self.enable_cli.call_async(req)
        self._wait_future(future, 2.0)
        return future.done() and future.result() is not None

    def call_recalibrate(self): return self._trigger(self.recalibrate_cli)
    def call_run_calibration(self): return self._trigger(self.run_calibration_cli)

    # Round 14 AA: LAB color helpers (vendor lab_manager parity).
    def lab_enter(self): return self._trigger_with_msg(self.lab_enter_cli)
    def lab_exit(self):  return self._trigger_with_msg(self.lab_exit_cli)
    def lab_save(self):  return self._trigger_with_msg(self.lab_save_to_disk_cli)

    def lab_get_range(self, color_name):
        req = self._lab_GetRange.Request()
        req.color_name = str(color_name)
        fut = self.lab_get_range_cli.call_async(req)
        deadline = time.time() + 3.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        if res is None or not getattr(res, 'success', False):
            return None
        return list(res.min), list(res.max)

    def lab_change_range(self, mn, mx):
        req = self._lab_ChangeRange.Request()
        req.min = [int(x) for x in mn]
        req.max = [int(x) for x in mx]
        fut = self.lab_change_range_cli.call_async(req)
        deadline = time.time() + 2.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        return bool(res and getattr(res, 'success', False))

    def lab_stash_range(self, color_name):
        req = self._lab_StashRange.Request()
        req.color_name = str(color_name)
        fut = self.lab_stash_range_cli.call_async(req)
        deadline = time.time() + 3.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        return bool(res and getattr(res, 'success', False))

    def lab_get_all_names(self):
        req = self._lab_GetAllColorName.Request()
        fut = self.lab_get_all_names_cli.call_async(req)
        deadline = time.time() + 3.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        if res is None:
            return []
        return list(res.color_names)

    def trigger_service(self, name):
        """Generic Trigger-service call by short name. Used by tabs that
        add buttons after the constructor (Round 12 Y6: depth_plane_refit)."""
        cli = getattr(self, f'{name}_cli', None)
        if cli is None:
            return False
        return self._trigger(cli)
    def call_reload_engine(self): return self._trigger(self.reload_engine_cli)
    def call_test_grip(self, class_name):
        return self._set_string_bool(self.test_grip_cli, str(class_name), True)
    def call_enter(self):       return self._trigger(self.enter_cli)
    def call_exit(self):        return self._trigger(self.exit_cli)
    def call_save_default(self): return self._trigger(self.save_default_cli)

    def call_save_yolo_config(self, cfg_dict, persist=True):
        """Apply (and optionally persist) the buffered Model-tab config."""
        return self._set_string_bool(self.save_yolo_cli, json.dumps(cfg_dict),
                                     bool(persist))

    def apply_and_persist(self, params, persist=True):
        """Apply a {param: value} dict and (default) persist it into
        default.yaml so it survives relaunch. Backs every per-tab Save&Apply."""
        return self._set_string_bool(self.apply_persist_cli, json.dumps(params),
                                     bool(persist))

    def call_load_engine(self, path):
        return self._set_string_bool(self.load_engine_cli, path, True)

    def call_save_profile(self, name):
        return self._set_string_bool(self.save_profile_cli, name, True)

    def call_load_profile(self, name):
        return self._set_string_bool(self.load_profile_cli, name, True)

    def call_teach(self, cmd, persist=False):
        """Send a bin-teach / workspace-teach command (a JSON dict) to the
        node. Returns True if the node accepted it; richer status (e.g. the
        workspace guardrail, the live coordinate) arrives on the heartbeat
        (last_teach_msg / teach_pose / teach_joints)."""
        return self._set_string_bool(self.teach_cli, json.dumps(cmd), bool(persist))


class TunerUI:
    def __init__(self, client, calib_only=None):
        self.client = client
        # Round 16 LL.4: when calib_only is 'position'/'color'/'depth' the
        # window shows ONLY the three calibration tabs (pop-out mode).
        self.calib_only = calib_only
        self._img_previews = []  # keep PhotoImage refs alive
        self.root = tk.Tk()
        self.root.title('JetArm v5 - calibration'
                        if calib_only else 'JetArm v5 - live tuner')
        # Fit the window to the screen so the bottom-pinned Save & Apply bars
        # and the presets bar are never pushed off the bottom of a small
        # Jetson display. Tab content itself scrolls (see _make_scrollable).
        try:
            sh = self.root.winfo_screenheight()
        except Exception:
            sh = 1100
        self.root.geometry(f'760x{min(1100, max(600, sh - 80))}')
        # name -> callable(value) that updates that param's widget locally
        # (no ROS traffic). Used by presets to keep the UI in sync.
        self._param_setters = {}
        # name -> callable() returning the current widget value (for per-tab
        # Save & Apply; read straight from the widget, no service round-trip).
        self._param_get = {}
        self._building = True
        self._build()
        self._building = False
        # Heartbeat mirror: poll the node's ~/status JSON once a second so
        # the perf label, engine label and RUNNING/STOPPED state reflect
        # reality (e.g. watchdog-stopped node while UI still said RUNNING).
        self.root.after(1000, self._poll_node_status)

    def _poll_node_status(self):
        try:
            st = self.client.latest_status
            self._last_status = st  # Round 12 X1: read in _on_calibrate
            age = time.time() - self.client.status_rx_t
            if st is None:
                pass  # node hasn't published yet - keep the placeholder
            elif age > 12.0:
                if hasattr(self, 'perf_var'):
                    self.perf_var.set(f'perf: NO HEARTBEAT for {age:.0f}s - node down?')
            else:
                unmapped = int(st.get('unmapped_count', 0) or 0)
                badge = f"  unmapped={unmapped}" if unmapped else ''
                # Depth status (Round 11): ON if a fresh frame arrived
                # recently, STALE if subscribed but no frame in >1s, OFF if
                # the subscription never connected.
                d_avail = bool(st.get('depth_available', False))
                d_age = int(st.get('depth_age_ms', -1) or -1)
                if not d_avail:
                    d_str = 'OFF'
                elif d_age < 0 or d_age > 1000:
                    d_str = 'STALE'
                else:
                    d_str = 'ON'
                if hasattr(self, 'perf_var'):
                    self.perf_var.set(
                        f"perf: cam={st.get('cam_fps', '-')}fps "
                        f"pub={st.get('pub_fps', '-')}fps "
                        f"ai={st.get('ai', '?')} "
                        f"inf_age={st.get('inference_age_ms', '-')}ms "
                        f"depth={d_str}{badge}")
                # Round 15: split per-tab status panels.
                tp = st.get('table_plane')
                plane_str = ('[{:.3f}, {:.3f}, {:.3f}, {:.3f}]'.format(*tp)
                             if tp and len(tp) == 4 else 'NOT FIT')
                tf_str = 'aligned' if st.get('depth_tf_ok') else 'unavailable'
                last = st.get('last_calibrate') or {}
                last_str = ('OK ({})'.format(last.get('source', '?'))
                            if last.get('ok')
                            else ('{}: {}'.format(
                                    last.get('source', '?'),
                                    last.get('error') or '-')
                                  if last else '(never)'))
                if hasattr(self, 'position_status_var'):
                    self.position_status_var.set(
                        f"last calibrate : {last_str}\n"
                        f"workspace size : "
                        f"{st.get('workspace_size_x', '?')} x "
                        f"{st.get('workspace_size_y', '?')} m"
                    )
                if hasattr(self, 'depth_status_var'):
                    self.depth_status_var.set(
                        f"depth topic : {st.get('depth_topic', '') or 'OFF'}\n"
                        f"depth status: {d_str} (age {d_age} ms)\n"
                        f"depth->color TF: {tf_str}\n"
                        f"table plane : {plane_str}"
                    )
                engine = st.get('engine') or ''
                task = st.get('task') or ''
                override = (st.get('task_override') or 'auto').strip().lower()
                if engine and hasattr(self, 'engine_var'):
                    self.engine_var.set(engine)
                    # Keep the Detection-tab "Active:" label + list marker in
                    # sync with the actually-loaded engine.
                    shown_key = (engine, task, override)
                    if hasattr(self, 'active_engine_var') \
                            and shown_key != getattr(self, '_active_engine_shown', None):
                        self._active_engine_shown = shown_key
                        if task and override and override != 'auto':
                            suffix = f' (task={task}, forced)'
                        elif task:
                            suffix = f' (task={task})'
                        else:
                            suffix = ''
                        self.active_engine_var.set(f'Active: {engine}{suffix}')
                        try:
                            self._refresh_engine_list(engine)
                        except Exception:
                            pass
                # Populate the Model class filter + Places rows from the
                # model's class names (this is the wiring that makes those
                # tabs work at all). Guards inside skip rebuilds while the
                # Model tab is dirty.
                cnames = st.get('class_names') or {}
                try:
                    self._refresh_class_filter(cnames)
                    self._refresh_places(cnames)
                except Exception:
                    pass
                # v5.1 Bin Teach tab: live readout (from ~/teach_status) +
                # the class dropdown populated from the model class names.
                if hasattr(self, 'teach_pose_var'):
                    try:
                        self._update_teach_readout(cnames)
                        self._refresh_teach_bins(cnames)
                    except Exception:
                        pass
                # Live grip telemetry on the Grip tab.
                try:
                    lg = st.get('last_grip')
                    running = bool(st.get('test_grip_running'))
                    if running:
                        self.grip_status_var.set('test grip running ...')
                    elif lg:
                        bits = [f"last grip [{lg.get('source','?')}]:",
                                str(lg.get('label', '?')),
                                str(lg.get('outcome', '?'))]
                        if lg.get('final_pulse') is not None:
                            bits.append(f"{lg['final_pulse']}p")
                        if lg.get('peak_temp') is not None:
                            bits.append(f"{lg['peak_temp']}C")
                        if lg.get('duration_ms') is not None:
                            bits.append(f"{lg['duration_ms']}ms")
                        self.grip_status_var.set(' '.join(bits))
                except Exception:
                    pass
                # Node state is authoritative for the steady RUNNING/STOPPED
                # display. Don't stomp the long-running CALIBRATING...
                # transient; short transients (PROFILE SAVED etc.) get
                # corrected on the next poll, which is the point.
                node_running = bool(st.get('enter')) and bool(st.get('sorting'))
                cur = self.status_var.get()
                if not self.calib_only and cur != 'CALIBRATING...':
                    if node_running and not cur.startswith('RUNNING'):
                        self._set_status('RUNNING (node)', '#2e8b57')
                    elif not node_running and cur.startswith('RUNNING'):
                        self._set_status('STOPPED (node)', '#aa3333')
        except Exception:
            pass  # never let the poll loop die
        self.root.after(1000, self._poll_node_status)

    def _update_teach_readout(self, class_names):
        """Refresh the Bin Teach tab from the low-latency ~/teach_status topic
        and keep its class dropdown in sync with the model's class names."""
        try:
            names = sorted(class_names.values()) if class_names else []
            if names and list(self.teach_class_combo['values']) != names:
                self.teach_class_combo['values'] = names
                if not self.teach_class_var.get():
                    self.teach_class_var.set(names[0])
        except Exception:
            pass
        tt = self.client.latest_teach
        if not tt:
            return
        # keep the Saved-bins map fresh (the node republishes it after each save)
        pp = tt.get('place_positions')
        if pp is not None:
            try:
                d = json.loads(pp or '{}')
                if isinstance(d, dict):
                    self._teach_places = d
            except Exception:
                pass
        pose = tt.get('teach_pose')
        if pose and len(pose) == 3:
            self.teach_pose_var.set(
                'world (x, y, z):  {:.4f},  {:.4f},  {:.4f}  m'.format(*pose))
        else:
            self.teach_pose_var.set('world (x, y, z): -- (jog or Refresh readout)')
        joints = tt.get('teach_joints')
        if joints:
            self.teach_joints_var.set(
                'servos 1-5:  ' + '   '.join(str(int(j)) for j in joints))
        center = tt.get('teach_center')
        edges = int(tt.get('teach_edges', 0) or 0)
        if center and len(center) == 3:
            self.teach_center_var.set(
                'workspace: centre {:.3f}, {:.3f}, {:.3f}  |  edges staged: {}'
                .format(center[0], center[1], center[2], edges))
        else:
            self.teach_center_var.set(
                f'workspace: centre not staged  |  edges staged: {edges}')
        running = bool(tt.get('teach_running'))
        msg = str(tt.get('last_teach_msg', '') or '')
        self.teach_msg_var.set(
            'status: ' + ('MOVING... ' if running else '') + (msg or 'idle'))

    def _refresh_teach_bins(self, class_names):
        """Keep the Bin Teach tab's 'Saved bins' list (one Go button + the live
        coordinate per class) in sync with the model classes + the current bin
        map (self._teach_places, refreshed from ~/teach_status)."""
        names = sorted(class_names.values()) if class_names else []
        if list(self._teach_bins_rows.keys()) != names:
            for w in self._teach_bins_inner.winfo_children():
                w.destroy()
            self._teach_bins_rows = {}
            if not names:
                ttk.Label(self._teach_bins_inner, foreground='#888',
                          text='(waiting for engine load...)').pack(anchor='w')
                return
            for name in names:
                row = ttk.Frame(self._teach_bins_inner); row.pack(fill='x', pady=2)
                ttk.Button(row, text='Go', width=5,
                           command=lambda nm=name: self._on_teach_goto_class(nm)
                           ).pack(side='left', padx=(0, 8))
                ttk.Label(row, text=name, width=16).pack(side='left')
                cv = tk.StringVar(value='(default)')
                ttk.Label(row, textvariable=cv, foreground='#3366aa').pack(side='left')
                self._teach_bins_rows[name] = cv
        places = getattr(self, '_teach_places', {}) or {}
        for name, cv in self._teach_bins_rows.items():
            v = places.get(name)
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                taught = len(v) >= 4 and str(v[3]) == 'taught'
                cv.set('{:.3f}, {:.3f}, {:.3f}   {}'.format(
                    float(v[0]), float(v[1]), float(v[2]),
                    '(taught)' if taught else '(set)'))
            else:
                cv.set('(default — Go still works)')

    def _build(self):
        if self.calib_only:
            self._build_calib_only()
            return
        # ---- Top control bar ----
        ctrl = ttk.LabelFrame(self.root, text='Robot control')
        ctrl.pack(fill='x', padx=8, pady=(8, 4))

        self.status_var = tk.StringVar(value='STOPPED')
        status_row = ttk.Frame(ctrl); status_row.pack(fill='x', padx=6, pady=4)
        ttk.Label(status_row, text='Status:').pack(side='left')
        self.status_label = tk.Label(status_row, textvariable=self.status_var,
                                     fg='white', bg='#aa3333',
                                     font=('TkDefaultFont', 11, 'bold'),
                                     padx=10, pady=2)
        self.status_label.pack(side='left', padx=8)

        self.engine_var = tk.StringVar(value='-')
        ttk.Label(status_row, text='   Engine:').pack(side='left')
        ttk.Label(status_row, textvariable=self.engine_var,
                  foreground='#3366aa').pack(side='left', padx=4)

        btn_row = ttk.Frame(ctrl); btn_row.pack(fill='x', padx=6, pady=4)
        self.start_btn = tk.Button(btn_row, text='START SORTING', bg='#2e8b57',
                                   fg='white', font=('TkDefaultFont', 11, 'bold'),
                                   width=16, height=2, command=self._on_start)
        self.start_btn.pack(side='left', padx=4)
        self.stop_btn = tk.Button(btn_row, text='STOP', bg='#aa3333',
                                  fg='white', font=('TkDefaultFont', 11, 'bold'),
                                  width=10, height=2, command=self._on_stop)
        self.stop_btn.pack(side='left', padx=4)
        self.cal_btn = tk.Button(btn_row, text='CALIBRATE', bg='#3366aa',
                                 fg='white', font=('TkDefaultFont', 11, 'bold'),
                                 width=12, height=2, command=self._on_calibrate)
        self.cal_btn.pack(side='left', padx=4)
        self.savedef_btn = tk.Button(btn_row, text='SAVE ALL AS DEFAULT', bg='#666688',
                                     fg='white', font=('TkDefaultFont', 10, 'bold'),
                                     width=18, height=2, command=self._on_save_default)
        self.savedef_btn.pack(side='left', padx=4)
        # Round 12 Z2: push the most recent session logs to the GitHub repo
        # so subsequent Claude Code sessions have full diagnostic context.
        self.pushlogs_btn = tk.Button(btn_row, text='PUSH LOGS', bg='#558855',
                                      fg='white',
                                      font=('TkDefaultFont', 10, 'bold'),
                                      width=11, height=2,
                                      command=self._on_push_logs)
        self.pushlogs_btn.pack(side='left', padx=4)

        # ---- Independent stop/start: camera subscription + YOLO inference ----
        # These let the user A/B the load contributions on the Orin Nano:
        # pause AI to see camera-only fps; pause camera to see the AI loop
        # run on the last frame; pause both for a baseline.
        toggle_row = ttk.Frame(ctrl); toggle_row.pack(fill='x', padx=6, pady=(0, 4))
        self.cam_toggle_btn = tk.Button(
            toggle_row, text='Pause camera', bg='#996633', fg='white',
            font=('TkDefaultFont', 10, 'bold'), width=14, height=2,
            command=self._on_toggle_camera_sub)
        self.cam_toggle_btn.pack(side='left', padx=4)
        self.ai_toggle_btn = tk.Button(
            toggle_row, text='Pause AI', bg='#664488', fg='white',
            font=('TkDefaultFont', 10, 'bold'), width=14, height=2,
            command=self._on_toggle_inference)
        self.ai_toggle_btn.pack(side='left', padx=4)
        # Live perf-status label: cam_fps / pub_fps / inference age.
        self.perf_var = tk.StringVar(value='perf: cam=- pub=- inf=-')
        ttk.Label(toggle_row, textvariable=self.perf_var,
                  foreground='#226666', font=('TkDefaultFont', 10)
                  ).pack(side='left', padx=12)

        ttk.Label(ctrl, foreground='#555',
                  text='Tip: STOP halts vision + motion. Pause camera/AI for '
                       'independent toggle without quitting. Adjust sliders '
                       'freely while stopped, then START.'
                  ).pack(anchor='w', padx=8, pady=(0, 4))

        # ---- Camera-view controls ----
        cam = ttk.LabelFrame(self.root, text='Camera view')
        cam.pack(fill='x', padx=8, pady=(0, 4))
        self.cam_status_var = tk.StringVar(value='closed')
        cam_status_row = ttk.Frame(cam); cam_status_row.pack(fill='x', padx=6, pady=4)
        ttk.Label(cam_status_row, text='Status:').pack(side='left')
        self.cam_status_label = tk.Label(cam_status_row,
                                         textvariable=self.cam_status_var,
                                         fg='white', bg='#666666',
                                         font=('TkDefaultFont', 10, 'bold'),
                                         padx=8, pady=2)
        self.cam_status_label.pack(side='left', padx=8)
        ttk.Label(cam_status_row, text='Topic:').pack(side='left', padx=(12, 4))
        self.cam_topic_entry = ttk.Entry(cam_status_row, width=42)
        self.cam_topic_entry.insert(0, '/custom_sortingv5/image_result')
        self.cam_topic_entry.pack(side='left', fill='x', expand=True)

        cam_btn_row = ttk.Frame(cam); cam_btn_row.pack(fill='x', padx=6, pady=4)
        tk.Button(cam_btn_row, text='Open rqt_image_view', bg='#2e8b57',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  width=18, height=2,
                  command=self._on_open_rqt).pack(side='left', padx=4)
        tk.Button(cam_btn_row, text='Open image_view', bg='#3366aa',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  width=16, height=2,
                  command=self._on_open_image_view).pack(side='left', padx=4)
        tk.Button(cam_btn_row, text='Open browser', bg='#774488',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  width=14, height=2,
                  command=self._on_open_browser).pack(side='left', padx=4)
        tk.Button(cam_btn_row, text='Close viewer', bg='#aa3333',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  width=14, height=2,
                  command=self._on_close_viewer).pack(side='left', padx=4)
        ttk.Label(cam, foreground='#555',
                  text='Each button replaces the currently open viewer with a '
                       'new one. Browser uses Hiwonder web_video_server, IP '
                       'detected via hostname -I.'
                  ).pack(anchor='w', padx=8, pady=(0, 4))

        # Track the currently spawned viewer subprocess (if any). Browser
        # opens are fire-and-forget (we don't try to close browser tabs).
        self._viewer_proc = None

        # ---- Tabs ----
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=8, pady=8)

        # v5 Round 2 tab layout. The Detection tab owns detection-lock params
        # AND the model (engine picker + live YOLO knobs + class checkboxes).
        # Each tab has a "Save & Apply" bar that applies live and persists to
        # default.yaml. Places = per-class targets.
        self._notebook = notebook
        speed_tab = ttk.Frame(notebook); notebook.add(speed_tab, text='Speed / motion')
        grip_tab = ttk.Frame(notebook); notebook.add(grip_tab, text='Grip')
        # v5 Round 2: the Model tab is gone — all model settings (engine
        # picker, YOLO knobs, class filter) now live on the Detection tab,
        # applied LIVE like every other slider and persisted by the tab's
        # "Save & Apply" button.
        detect_tab = ttk.Frame(notebook); notebook.add(detect_tab, text='Detection')
        self._model_tab = detect_tab
        self._model_tab_index = notebook.index('end') - 1
        places_tab = ttk.Frame(notebook); notebook.add(places_tab, text='Places')
        # v5.1 bin-teach: jog the arm to each physical bin (joint-by-joint or
        # world XYZ), read the live coordinate, and save it per class so
        # everything sorts to the right place. Also teach the workspace
        # centre/edges to re-anchor the world map.
        teach_tab = ttk.Frame(notebook); notebook.add(teach_tab, text='Bin Teach')
        # Round 15: three independent calibration tabs - each with its own
        # CALIBRATE button that ONLY runs that calibration. Position drives
        # the vendor calibration_node; Color drives the vendor lab_manager;
        # Depth drives our depth_plane_refit (which uses vendor SearchPlane).
        position_tab = ttk.Frame(notebook); notebook.add(position_tab, text='Position')
        color_tab = ttk.Frame(notebook); notebook.add(color_tab, text='Color')
        depth_tab = ttk.Frame(notebook); notebook.add(depth_tab, text='Depth')
        toggles_tab = ttk.Frame(notebook); notebook.add(toggles_tab, text='Toggles')
        profiles_tab = ttk.Frame(notebook); notebook.add(profiles_tab, text='Profiles')

        # Live grip telemetry pinned at the top of the Grip tab (stays visible
        # while the rest of the tab scrolls).
        self.grip_status_var = tk.StringVar(value='last grip: --')
        ttk.Label(grip_tab, textvariable=self.grip_status_var,
                  foreground='#226666',
                  font=('TkDefaultFont', 9, 'bold')
                  ).pack(anchor='w', padx=8, pady=(8, 0))
        ttk.Label(grip_tab, foreground='#666', wraplength=720,
                  justify='left',
                  text='BETA force-limited grasp (enable on Toggles tab): closes '
                       'until jaws contact the item. Per-class max strength '
                       'caps the squeeze (set in Places). Servo temp cutoff at '
                       'grasp_max_temp. Use the "Test grip" buttons in Places '
                       'to tune each class’ strength without firing a full pick.'
                  ).pack(anchor='w', padx=8, pady=(0, 6))

        all_names = ([n for n, *_ in FLOAT_PARAMS] + [n for n, *_ in INT_PARAMS]
                     + [n for n, *_ in MODEL_FLOAT_PARAMS]
                     + [n for n, *_ in MODEL_INT_PARAMS]
                     + [n for n, *_ in CALIB_FLOAT_PARAMS]
                     + BOOL_PARAMS + ['engine_path', 'engine_task',
                                       'yolo_enabled_classes',
                                       'place_positions',
                                       'calibrate_overlay_mode'])
        current = self.client.get_values(all_names)
        self.engine_var.set(self._short_engine(current.get('engine_path', '-')))

        speed_names = {'motion_speed', 'aggression', 'hover_height',
                       'approach_dwell', 'gripper_close_duration'}
        grip_names = {'gripper_open_pulse', 'gripper_close_pulse',
                      'gripper_settle', 'grab_depth',
                      'grasp_step_pulse', 'grasp_step_dwell',
                      'grasp_stall_pulse', 'grasp_timeout'}
        detect_keys = ([n for n, *_ in FLOAT_PARAMS
                        if n not in speed_names and n not in grip_names]
                       + [n for n, *_ in INT_PARAMS if n not in grip_names]
                       + [n for n, *_ in MODEL_FLOAT_PARAMS]
                       + [n for n, *_ in MODEL_INT_PARAMS])

        # Per-tab "Save & Apply" bars are pinned to the BOTTOM of each tab
        # (outside the scroll region) so they're always reachable. Add them
        # first so the scrollable body fills the space above them.
        self._add_tab_save_bar(speed_tab, sorted(speed_names), 'Speed')
        self._add_tab_save_bar(grip_tab, sorted(grip_names), 'Grip')
        self._add_tab_save_bar(detect_tab, detect_keys, 'Detection',
                               get_extra=self._detect_save_extra)
        self._add_tab_save_bar(toggles_tab, list(BOOL_PARAMS), 'Toggles')
        self._add_tab_save_bar(places_tab,
                               ['place_positions', 'grasp_strength'], 'Places')
        # Round 15: Position + Depth tabs each get their own Save & Close
        # bar. Position persists the world XY offset + workspace size;
        # Depth persists the depth tunables. Bodies added below.
        position_keys = [n for n, *_ in CALIB_FLOAT_PARAMS]
        self._add_calibrate_save_bar(position_tab, position_keys)
        depth_keys = ([n for n, *_ in CALIB_DEPTH_FLOAT_PARAMS]
                      + [n for n, *_ in CALIB_DEPTH_INT_PARAMS]
                      + ['use_depth_for_z', 'overlay_depth_view'])
        self._add_calibrate_save_bar(depth_tab, depth_keys)

        # Scrollable content bodies so nothing is clipped on a small Jetson
        # screen (this is the fix for "I can't scroll" / "can't reach the
        # engine picker"). Content goes into the *body*, not the tab frame.
        speed_body = self._make_scrollable(speed_tab)
        grip_body = self._make_scrollable(grip_tab)
        detect_body = self._make_scrollable(detect_tab)
        toggles_body = self._make_scrollable(toggles_tab)
        places_body = self._make_scrollable(places_tab)
        teach_body = self._make_scrollable(teach_tab)
        position_body = self._make_scrollable(position_tab)
        color_body = self._make_scrollable(color_tab)
        depth_body = self._make_scrollable(depth_tab)

        # Everything not speed/grip lives on the Detection tab.
        for name, lo, hi, res in FLOAT_PARAMS:
            parent = (speed_body if name in speed_names else
                      grip_body if name in grip_names else detect_body)
            self._add_float(parent, name, lo, hi, res, current.get(name, lo))
        for name, lo, hi, res in INT_PARAMS:
            parent = grip_body if name in grip_names else detect_body
            self._add_int(parent, name, lo, hi, res, int(current.get(name, lo)))
        for name in BOOL_PARAMS:
            self._add_bool(toggles_body, name, bool(current.get(name, False)))

        # Detection tab also owns the model: live YOLO knobs + engine picker +
        # class filter (built into the scrollable body).
        self._build_model_section(detect_body, current)
        # Places tab — per-class targets (place position + grip strength).
        self._build_places_tab(places_body, current.get('place_positions', '{}'))
        # Bin Teach tab — jog the arm to each bin, read the coordinate, save it.
        self._build_teach_tab(teach_body, current.get('place_positions', '{}'))
        # Round 15: three independent calibration tabs.
        self._build_position_tab(position_body, current)
        self._build_color_tab(color_body, current)
        self._build_depth_tab(depth_body, current)
        # Profiles tab is short and has its own listbox scroll - no body wrap.
        self._build_profiles_tab(profiles_tab)

        # Presets bar: built-in quick presets + namable custom presets.
        preset = ttk.LabelFrame(self.root, text='Presets')
        preset.pack(fill='x', padx=8, pady=4)
        for _label, _slug in SPEED_PRESETS:
            ttk.Button(preset, text=_label,
                       command=lambda s=_slug: self._on_load_speed_preset(s)
                       ).pack(side='left', padx=4, pady=4)
        # Namable custom presets (saved as full profiles in PROFILES_DIR).
        ttk.Separator(preset, orient='vertical').pack(side='left', fill='y',
                                                      padx=8, pady=4)
        ttk.Label(preset, text='custom:').pack(side='left', padx=(4, 2))
        self._preset_name_entry = ttk.Entry(preset, width=14)
        self._preset_name_entry.pack(side='left', padx=2)
        ttk.Button(preset, text='Save as preset',
                   command=self._on_save_preset).pack(side='left', padx=2)
        self._preset_combo = ttk.Combobox(preset, width=14, state='readonly')
        self._preset_combo.pack(side='left', padx=(8, 2))
        ttk.Button(preset, text='Load preset',
                   command=self._on_load_preset).pack(side='left', padx=2)
        self._refresh_preset_combo()

    def _build_calib_only(self):
        """Round 16 LL.4: pop-out window showing ONLY the Position / Color /
        Depth calibration tabs. A normal service client of the running
        nodes on the same domain - no sorting, no camera open."""
        bar = ttk.Frame(self.root); bar.pack(fill='x', padx=8, pady=(8, 0))
        ttk.Label(bar, text='Calibration tools (separate window, '
                            'same domain)',
                  font=('TkDefaultFont', 10, 'bold')).pack(side='left')
        self.status_var = tk.StringVar(value='ready')
        self.status_label = tk.Label(self.root, textvariable=self.status_var,
                                     bg='#dddddd', anchor='w')
        self.status_label.pack(fill='x', padx=8, pady=(2, 4))

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=8, pady=8)
        self._notebook = notebook
        position_tab = ttk.Frame(notebook); notebook.add(position_tab, text='Position')
        color_tab = ttk.Frame(notebook); notebook.add(color_tab, text='Color')
        depth_tab = ttk.Frame(notebook); notebook.add(depth_tab, text='Depth')

        all_names = ([n for n, *_ in CALIB_FLOAT_PARAMS]
                     + [n for n, *_ in CALIB_DEPTH_FLOAT_PARAMS]
                     + [n for n, *_ in CALIB_DEPTH_INT_PARAMS]
                     + ['use_depth_for_z', 'overlay_depth_view'])
        current = self.client.get_values(all_names)

        position_keys = [n for n, *_ in CALIB_FLOAT_PARAMS]
        self._add_calibrate_save_bar(position_tab, position_keys)
        depth_keys = ([n for n, *_ in CALIB_DEPTH_FLOAT_PARAMS]
                      + [n for n, *_ in CALIB_DEPTH_INT_PARAMS]
                      + ['use_depth_for_z', 'overlay_depth_view'])
        self._add_calibrate_save_bar(depth_tab, depth_keys)

        position_body = self._make_scrollable(position_tab)
        color_body = self._make_scrollable(color_tab)
        depth_body = self._make_scrollable(depth_tab)
        self._build_position_tab(position_body, current)
        self._build_color_tab(color_body, current)
        self._build_depth_tab(depth_body, current)

        # Raise the requested tab.
        idx = {'position': 0, 'color': 1, 'depth': 2}.get(self.calib_only, 0)
        try:
            notebook.select(idx)
        except Exception:
            pass
        self.root.after(1000, self._poll_node_status)

    def _short_engine(self, path):
        if not path or path == '-': return '-'
        return os.path.basename(path)

    # ---- scrollable tab bodies ----

    def _make_scrollable(self, parent):
        """Return an inner ttk.Frame inside a vertically-scrollable canvas.
        Pack your tab content into the returned frame. A draggable scrollbar
        plus mousewheel/touch scrolling keep tall tabs reachable on the small
        Jetson screen. The canvas fills the space ABOVE any bottom-pinned bar
        already packed on `parent`."""
        canvas = tk.Canvas(parent, highlightthickness=0)
        vsb = ttk.Scrollbar(parent, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        inner = ttk.Frame(canvas)
        win = canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>',
                   lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        # Keep the inner frame the same width as the canvas so sliders fill it.
        canvas.bind('<Configure>',
                    lambda e: canvas.itemconfigure(win, width=e.width))
        # Register for the single global mousewheel handler (routes to the
        # canvas under the pointer, so wheel works over child sliders too).
        if not hasattr(self, '_scroll_canvases'):
            self._scroll_canvases = []
            self.root.bind_all('<MouseWheel>', self._on_global_wheel, add='+')
            self.root.bind_all('<Button-4>', self._on_global_wheel, add='+')
            self.root.bind_all('<Button-5>', self._on_global_wheel, add='+')
        self._scroll_canvases.append(canvas)
        return inner

    def _on_global_wheel(self, e):
        """Scroll whichever registered canvas is under the pointer. Handles
        both <MouseWheel> (Win/Mac, e.delta) and <Button-4/5> (X11/Jetson)."""
        try:
            w = self.root.winfo_containing(e.x_root, e.y_root)
        except Exception:
            return
        while w is not None:
            if w in getattr(self, '_scroll_canvases', ()):
                num = getattr(e, 'num', None)
                delta = getattr(e, 'delta', 0)
                step = -1 if (num == 4 or delta > 0) else 1
                w.yview_scroll(step, 'units')
                return
            w = getattr(w, 'master', None)

    # ---- generic widgets ----

    def _add_float(self, parent, name, lo, hi, res, init):
        self._add_numeric(parent, name, lo, hi, res, init, kind='float')

    def _add_int(self, parent, name, lo, hi, res, init):
        self._add_numeric(parent, name, lo, hi, res, init, kind='int')

    def _attach_tooltip(self, widget, text):
        """Lightweight hover tooltip (no external deps)."""
        tip = {'win': None}

        def show(_e):
            if tip['win'] is not None:
                return
            x = widget.winfo_rootx() + 20
            y = widget.winfo_rooty() + widget.winfo_height() + 2
            win = tk.Toplevel(widget)
            win.wm_overrideredirect(True)
            win.wm_geometry(f'+{x}+{y}')
            tk.Label(win, text=text, justify='left', bg='#ffffe0',
                     relief='solid', borderwidth=1,
                     font=('TkDefaultFont', 8)).pack()
            tip['win'] = win

        def hide(_e):
            if tip['win'] is not None:
                tip['win'].destroy(); tip['win'] = None

        widget.bind('<Enter>', show)
        widget.bind('<Leave>', hide)

    def _add_numeric(self, parent, name, lo, hi, res, init, kind):
        """One slider row with:
          - label (name)
          - slider that updates the local display on drag, but only
            pushes to ROS on mouse-release (sliderReleased() pattern).
            Stops the executor saturation we saw in the v4.1 logs (pre-v5)
            (hundreds of /set_parameters calls per drag).
          - numeric entry box for exact values (type then Enter or tab).
            Both paths produce exactly one ROS call per finalized value.
        """
        frame = ttk.Frame(parent); frame.pack(fill='x', padx=6, pady=3)
        disp, help_txt = PARAM_LABELS.get(name, (name, ''))
        lbl = ttk.Label(frame, text=disp, width=28)
        lbl.pack(side='left')
        if help_txt:
            try:
                self._attach_tooltip(lbl, f'{name}\n{help_txt}')
            except Exception:
                pass

        fmt = (lambda v: f'{float(v):.3f}') if kind == 'float' else (lambda v: f'{int(round(float(v)))}')
        var = tk.DoubleVar(value=float(init)) if kind == 'float' else tk.IntVar(value=int(init))

        # Right-hand numeric entry box
        entry = ttk.Entry(frame, width=8, justify='right')
        entry.insert(0, fmt(init))
        entry.pack(side='right')

        def push_to_ros(v):
            """Send a single /set_parameters call. Always clamps + formats."""
            if self._building:
                return
            try:
                v = float(v) if kind == 'float' else int(round(float(v)))
            except (TypeError, ValueError):
                return
            v = max(lo, min(hi, v))
            var.set(v)
            entry.delete(0, 'end'); entry.insert(0, fmt(v))
            self.client.set_value(name, v)

        def on_drag(value_str):
            """Drag: update entry display LOCALLY only. No ROS traffic."""
            try:
                v = float(value_str)
            except (TypeError, ValueError):
                return
            if kind == 'int':
                v = int(round(v))
            entry.delete(0, 'end'); entry.insert(0, fmt(v))

        def on_release(_event):
            push_to_ros(var.get())

        def on_entry_commit(_event=None):
            push_to_ros(entry.get())

        scale = ttk.Scale(frame, from_=lo, to=hi, variable=var,
                          orient='horizontal', command=on_drag)
        scale.pack(side='left', fill='x', expand=True, padx=8)
        # Only push on mouse-release - the key decoupling.
        scale.bind('<ButtonRelease-1>', on_release)

        # Numeric entry: Enter or focus-out commits.
        entry.bind('<Return>', on_entry_commit)
        entry.bind('<FocusOut>', on_entry_commit)
        # Up/Down arrows nudge by `res`.
        def _nudge(sign):
            try:
                v = float(entry.get())
            except ValueError:
                v = float(init)
            v = v + sign * float(res or (1 if kind == 'int' else 0.01))
            push_to_ros(v)
        entry.bind('<Up>',   lambda e: _nudge(+1))
        entry.bind('<Down>', lambda e: _nudge(-1))

        def set_local(v):
            """Update slider + entry display without ROS traffic."""
            try:
                v = float(v) if kind == 'float' else int(round(float(v)))
            except (TypeError, ValueError):
                return
            v = max(lo, min(hi, v))
            var.set(v)
            entry.delete(0, 'end'); entry.insert(0, fmt(v))
        self._param_setters[name] = set_local
        # Getter for per-tab Save & Apply: read the CURRENT widget value
        # directly (WYSIWYG, no service round-trip). The Scale tracks `var`
        # live, so this reflects exactly what the user sees.
        if not hasattr(self, '_param_get'):
            self._param_get = {}
        self._param_get[name] = (
            (lambda: max(lo, min(hi, float(var.get())))) if kind == 'float'
            else (lambda: int(max(lo, min(hi, round(float(var.get())))))))

    def _add_bool(self, parent, name, init):
        var = tk.BooleanVar(value=bool(init))
        def on_change():
            if not self._building:
                self.client.set_value(name, bool(var.get()))
        ttk.Checkbutton(parent, text=name, variable=var, command=on_change
                        ).pack(anchor='w', padx=12, pady=4)
        self._param_setters[name] = lambda v: var.set(bool(v))
        if not hasattr(self, '_param_get'):
            self._param_get = {}
        self._param_get[name] = lambda: bool(var.get())

    # ---- Model section (built into the Detection tab; live + Save & Apply) ----

    def _build_model_section(self, parent, current):
        """Model controls folded into the Detection tab: LIVE YOLO knobs +
        engine picker (Apply/Reload that actually switch) + per-class enable
        checkboxes. Knobs apply on release like every other slider; the engine
        and class filter apply on their buttons / the tab's Save & Apply.
        Persistence is the tab Save & Apply (merges into default.yaml)."""
        self._model_dirty = False
        self._model_class_vars = {}       # class_name -> tk.BooleanVar
        self._model_class_id_map = {}     # class_name -> int id
        self._model_class_widgets = []    # (frame, name) for the filter

        # --- Live YOLO knobs (conf / iou / max_det / inference_hz) ---
        knob_frame = ttk.LabelFrame(parent, text='Model — YOLO detection knobs '
                                                 '(apply live on release)')
        knob_frame.pack(fill='x', padx=8, pady=(8, 4))
        for name, lo, hi, res in MODEL_FLOAT_PARAMS:
            self._add_float(knob_frame, name, lo, hi, res,
                            float(current.get(name, lo)))
        for name, lo, hi, res in MODEL_INT_PARAMS:
            self._add_int(knob_frame, name, lo, hi, res,
                          int(current.get(name, lo)))
        ttk.Button(knob_frame, text='Reset knobs to defaults',
                   command=self._on_reset_model_defaults).pack(anchor='w',
                                                              padx=6, pady=4)

        # --- Engine picker (Apply/Reload commit + switch the running model) ---
        eng_frame = ttk.LabelFrame(parent, text='Engine (.engine / .pt)')
        eng_frame.pack(fill='x', padx=8, pady=4)
        self._active_engine = str(current.get('engine_path', ''))
        # ACTIVE engine, driven live from ~/status so you always see which
        # model is actually loaded.
        self.active_engine_var = tk.StringVar(
            value=f'Active: {self._short_engine(self._active_engine)}')
        ttk.Label(eng_frame, textvariable=self.active_engine_var,
                  foreground='#2e8b57', font=('TkDefaultFont', 9, 'bold')
                  ).pack(anchor='w', padx=6, pady=(4, 0))
        lb_frame = ttk.Frame(eng_frame); lb_frame.pack(fill='x', padx=4, pady=4)
        self.engine_listbox = tk.Listbox(lb_frame, height=5)
        self.engine_listbox.pack(side='left', fill='x', expand=True)
        sb = ttk.Scrollbar(lb_frame, orient='vertical',
                           command=self.engine_listbox.yview)
        sb.pack(side='right', fill='y')
        self.engine_listbox.configure(yscrollcommand=sb.set)
        # Double-click a model to swap to it immediately (like the old flow).
        self.engine_listbox.bind('<Double-Button-1>',
                                 lambda e: self._on_pick_and_apply_engine())
        self._refresh_engine_list(self._active_engine)
        ebtn = ttk.Frame(eng_frame); ebtn.pack(fill='x', padx=4, pady=2)
        ttk.Button(ebtn, text='Refresh',
                   command=lambda: self._refresh_engine_list(self._active_engine)
                   ).pack(side='left', padx=4)
        ttk.Button(ebtn, text='Use selected',
                   command=self._on_pick_selected_engine).pack(side='left', padx=4)
        ttk.Button(ebtn, text='Browse...',
                   command=self._on_browse_engine).pack(side='left', padx=4)
        ttk.Button(ebtn, text='Apply engine',
                   command=self._on_apply_engine).pack(side='left', padx=4)
        ttk.Button(ebtn, text='Reload engine',
                   command=self._on_reload_engine).pack(side='left', padx=4)
        erow = ttk.Frame(eng_frame); erow.pack(fill='x', padx=4, pady=2)
        ttk.Label(erow, text='path', width=6).pack(side='left')
        self._model_engine_entry = ttk.Entry(erow)
        self._model_engine_entry.pack(side='left', fill='x', expand=True, padx=4)
        self._model_engine_entry.insert(0, self._active_engine)
        # Task override - lives next to the engine path so the user can mark
        # each engine as obb/detect/segment/pose/classify. Persists via the
        # Detection-tab Save & Apply alongside engine_path.
        trow = ttk.Frame(eng_frame); trow.pack(fill='x', padx=4, pady=2)
        ttk.Label(trow, text='task', width=6).pack(side='left')
        self._engine_task_var = tk.StringVar(
            value=str(current.get('engine_task', 'auto')))
        ttk.OptionMenu(trow, self._engine_task_var,
                       self._engine_task_var.get(),
                       'auto', 'detect', 'obb', 'segment', 'pose', 'classify',
                       command=lambda _v: self._mark_model_dirty()
                       ).pack(side='left', padx=4)
        ttk.Label(trow, foreground='#666',
                  text='set this if the engine was exported without task '
                       'metadata (e.g. ultralytics warned task=detect for '
                       'an obb model)').pack(side='left', padx=6)
        ttk.Label(eng_frame, foreground='#666',
                  text='Pick/Browse fills the path. "Apply engine" switches the '
                       'running model now; "Reload engine" re-inits it. The tab '
                       'Save & Apply persists the engine + task as the boot '
                       'defaults.'
                  ).pack(anchor='w', padx=6, pady=(0, 2))

        # --- Classes filter (plain frame; the whole Detection tab scrolls) ---
        cls_frame = ttk.LabelFrame(parent, text='Enabled YOLO classes '
                                                 '(none ticked = all classes)')
        cls_frame.pack(fill='x', padx=8, pady=(8, 4))
        ftop = ttk.Frame(cls_frame); ftop.pack(fill='x', padx=4, pady=2)
        ttk.Label(ftop, text='filter:').pack(side='left')
        self._class_filter_entry = ttk.Entry(ftop, width=18)
        self._class_filter_entry.pack(side='left', padx=4)
        self._class_filter_entry.bind('<KeyRelease>', lambda e: self._apply_class_filter_text())
        ttk.Button(ftop, text='All', width=5,
                   command=lambda: self._set_all_classes(True)).pack(side='left', padx=2)
        ttk.Button(ftop, text='None', width=5,
                   command=lambda: self._set_all_classes(False)).pack(side='left', padx=2)
        ttk.Button(ftop, text='Invert', width=6,
                   command=self._invert_classes).pack(side='left', padx=2)
        # No inner canvas/scroll here - the outer tab scroll (see
        # _make_scrollable) handles overflow; nesting scrolls is janky.
        self._model_classes_inner = ttk.Frame(cls_frame)
        self._model_classes_inner.pack(fill='x', padx=4, pady=2)
        ttk.Label(self._model_classes_inner, foreground='#888',
                  text='(waiting for engine load to read model.names...)'
                  ).pack(anchor='w', padx=4, pady=2)

    # ---- dirty tracking (Detection tab class-filter edits) ----

    def _mark_model_dirty(self):
        if getattr(self, '_building', False):
            return
        self._model_dirty = True
        try:
            self._notebook.tab(self._model_tab_index, text='Detection *')
        except Exception:
            pass

    def _clear_model_dirty(self):
        self._model_dirty = False
        try:
            self._notebook.tab(self._model_tab_index, text='Detection')
        except Exception:
            pass

    # ---- per-tab Save & Apply ----

    def _add_tab_save_bar(self, parent, keys, label, get_extra=None):
        """Right-aligned "Save & Apply" button at the bottom of a tab. Applies
        the tab's params live AND persists them to default.yaml (boot
        defaults). get_extra() optionally returns more {param: value} (used by
        the Detection tab for engine_path + the class filter)."""
        bar = ttk.Frame(parent)
        bar.pack(side='bottom', fill='x', padx=8, pady=6)
        tk.Button(bar, text='Save & Apply', bg='#2e8b57', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=lambda: self._on_tab_save(keys, label, get_extra)
                  ).pack(side='right', padx=4)
        ttk.Label(bar, foreground='#666',
                  text='applies now + saves to boot defaults (default.yaml)'
                  ).pack(side='right', padx=8)

    def _on_tab_save(self, keys, label, get_extra=None):
        # Build the payload from the WIDGETS directly (no get_values service
        # round-trip - that was returning {} and silently saving nothing).
        getters = getattr(self, '_param_get', {})
        vals = {}
        for k in (keys or []):
            if k in getters:
                try:
                    vals[k] = getters[k]()
                except Exception:
                    pass
            elif k == 'place_positions':
                vals[k] = json.dumps(getattr(self, '_places', {}) or {})
            elif k == 'grasp_strength':
                vals[k] = json.dumps(getattr(self, '_grasp_strength', {}) or {})
        if get_extra is not None:
            try:
                vals.update(get_extra() or {})
            except Exception:
                pass

        def go():
            ok = bool(vals) and self.client.apply_and_persist(vals, persist=True)
            if ok:
                self._set_status(f'{label} SAVED', '#3366aa')
                if label == 'Detection':
                    self._clear_model_dirty()
            elif not vals:
                messagebox.showinfo('Nothing to save',
                                    f'No editable settings on the {label} tab.')
            else:
                messagebox.showerror('Save failed',
                                     'apply_and_persist service rejected.')
        threading.Thread(target=go, daemon=True).start()

    # ---- Calibrate tab: manual workspace nudge + overlay ----

    def _add_calibrate_save_bar(self, parent, keys):
        """Save & Close button for the Calibrate tab. Persists the world XY
        offset + workspace scale to default.yaml AND clears the live
        calibration overlay (so the user sees a clean view post-save)."""
        bar = ttk.Frame(parent)
        bar.pack(side='bottom', fill='x', padx=8, pady=6)
        tk.Button(bar, text='Save & Close', bg='#2e8b57', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=lambda: self._on_calibrate_save_close(keys)
                  ).pack(side='right', padx=4)
        ttk.Label(bar, foreground='#666',
                  text='applies + saves to default.yaml + clears the overlay'
                  ).pack(side='right', padx=8)

    def _on_calibrate_save_close(self, keys):
        """Calibrate-tab save: persist values then flip overlay mode to off."""
        def go():
            getters = getattr(self, '_param_get', {})
            vals = {k: getters[k]() for k in (keys or []) if k in getters}
            ok = bool(vals) and self.client.apply_and_persist(vals,
                                                              persist=True)
            try:
                self.client.set_value('calibrate_overlay_mode', 'off')
            except Exception:
                pass
            if ok:
                self._set_status('Calibrate SAVED', '#3366aa')
            elif not vals:
                messagebox.showinfo('Nothing to save',
                                    'No calibrate settings to save.')
            else:
                messagebox.showerror('Save failed',
                                     'apply_and_persist service rejected.')
        threading.Thread(target=go, daemon=True).start()

    def _make_image_preview(self, parent, topic, label_text, mono=False,
                            size=(320, 240)):
        """Round 16 LL.1: live ROS image preview, reproducing the vendor
        web tool's video panel by subscribing to the same image topic the
        web_video_server streamed. Cheap: depth=1 QoS, <=10 Hz blit,
        bounded queue drops stale frames. Degrades to a text label if
        Pillow isn't installed."""
        frame = ttk.LabelFrame(parent, text=label_text)
        frame.pack(fill='x', padx=8, pady=4)
        try:
            from PIL import Image, ImageTk  # noqa: F401
        except Exception:
            ttk.Label(frame, foreground='#aa6633',
                      text='(install python3-pil.imagetk for the live '
                           'preview: sudo apt install python3-pil.imagetk)'
                      ).pack(anchor='w', padx=8, pady=6)
            return
        lbl = tk.Label(frame, width=size[0], height=size[1], bg='#222')
        lbl.pack(padx=6, pady=6)
        state = {'latest': None, 'last_blit': 0.0}

        try:
            from sensor_msgs.msg import Image as RosImage
        except Exception:
            ttk.Label(frame, text='(sensor_msgs unavailable)').pack()
            return

        def _on_img(msg):
            state['latest'] = msg  # background spin thread; just stash

        self.client.create_subscription(RosImage, topic, _on_img, 1)

        def _poll():
            import numpy as np
            from PIL import Image, ImageTk
            msg = state['latest']
            now = time.time()
            if msg is not None and (now - state['last_blit']) >= 0.1:
                state['last_blit'] = now
                try:
                    h, w = msg.height, msg.width
                    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                    enc = (msg.encoding or '').lower()
                    if mono or enc in ('mono8', '8uc1'):
                        arr = buf.reshape(h, w)
                        im = Image.fromarray(arr, 'L')
                    elif enc in ('rgb8',):
                        im = Image.fromarray(buf.reshape(h, w, 3), 'RGB')
                    else:  # bgr8 default
                        arr = buf.reshape(h, w, 3)[:, :, ::-1]
                        im = Image.fromarray(arr, 'RGB')
                    im = im.resize(size)
                    photo = ImageTk.PhotoImage(im)
                    lbl.configure(image=photo)
                    lbl.image = photo
                except Exception:
                    pass
            self.root.after(100, _poll)

        self.root.after(200, _poll)

    def _spawn_calibration_window(self, focus_tab):
        """Round 16 LL.4: open the calibration tabs in a SEPARATE terminal
        + window on the same ROS_DOMAIN_ID. It's a normal service client of
        the already-running nodes - no second camera open, no second
        sorting node."""
        def go():
            node = getattr(self.client, 'target', 'custom_sortingv5')
            inner = (f"ROS_DOMAIN_ID=0 ros2 run app tune_uiv5 "
                     f"--node-name {node} --calib-window={focus_tab}")
            for term in ('gnome-terminal', 'x-terminal-emulator',
                         'lxterminal', 'xfce4-terminal', 'xterm'):
                if shutil.which(term):
                    try:
                        if term == 'gnome-terminal':
                            subprocess.Popen([term, '--', 'bash', '-lc',
                                              inner + '; exec bash'])
                        else:
                            subprocess.Popen([term, '-e',
                                              f"bash -lc '{inner}; exec bash'"])
                        self._set_status(
                            f'Opened calibration window ({focus_tab})',
                            '#3366aa')
                        _ui_log('calib_window', f'spawned {term} -> {focus_tab}')
                        return
                    except Exception as e:
                        _ui_log('calib_window', f'{term} failed', exc=e)
                        continue
            # No terminal emulator: fall back to a detached window (no term).
            try:
                subprocess.Popen(['ros2', 'run', 'app', 'tune_uiv5',
                                  '--node-name', node,
                                  f'--calib-window={focus_tab}'],
                                 env={**os.environ, 'ROS_DOMAIN_ID': '0'})
                self._set_status(f'Opened calibration window ({focus_tab})',
                                 '#3366aa')
            except Exception as e:
                self._set_status('Could not open calibration window', '#aa6633')
                _ui_log('calib_window', 'all spawn methods failed', exc=e)
        threading.Thread(target=go, daemon=True).start()

    def _build_position_tab(self, parent, current):
        """Round 15: Position calibration tab - AprilTag → workspace
        world frame, drives the vendor calibration_node ONLY. The big
        CALIBRATE POSITION button is the only entrypoint to the
        AprilTag flow.

        Knobs (world XY offset, workspace size + scale) live here for
        manual-mode fine-tuning AFTER AprilTag calibration."""
        header = ttk.LabelFrame(parent, text='Position calibration')
        header.pack(fill='x', padx=8, pady=(8, 4))
        ttk.Label(header, foreground='#444', wraplength=720,
                  justify='left',
                  text=(
                      'POSITION CALIBRATION (AprilTag → workspace world frame)\n'
                      '• Drives the vendor calibration_node (same one the '
                      'Hiwonder PC tool uses).\n'
                      '• Requires ONE tag in [1, 2, 3, 100], 2.5 cm, '
                      'tag36h11, flat on the mat.\n'
                      '• On success the workspace overlay flashes briefly '
                      'so you see how it landed.\n'
                      '\n'
                      'AFTER calibration, fine-tune live:\n'
                      '• Cyan + at "0,0" is the workspace centre.\n'
                      '• Red/Green arrows are world +X (right) / +Y '
                      '(forward).\n'
                      '• Yellow rectangle = the workspace the arm thinks '
                      'it has. Size it to your mat with Workspace size '
                      'X / Y.\n'
                      '• Yellow X on each object = where the arm will '
                      'grab. When calibration is right, X sits on the '
                      'object centre.\n'
                      '\n'
                      'NO TAG? Click "Enter manual mode" and drag the '
                      'sliders by hand. Save & Close persists to '
                      'default.yaml.'
                  )).pack(anchor='w', padx=8, pady=(2, 6))

        knob_frame = ttk.LabelFrame(parent, text='World offsets + workspace size (live)')
        knob_frame.pack(fill='x', padx=8, pady=4)
        for name, lo, hi, res in CALIB_FLOAT_PARAMS:
            self._add_float(knob_frame, name, lo, hi, res,
                            float(current.get(name, lo)))

        # Status panel - position-only fields from heartbeat last_calibrate.
        status_frame = ttk.LabelFrame(parent,
                                      text='Position status (live)')
        status_frame.pack(fill='x', padx=8, pady=4)
        self.position_status_var = tk.StringVar(value='(awaiting heartbeat...)')
        ttk.Label(status_frame, textvariable=self.position_status_var,
                  foreground='#222', wraplength=720, justify='left',
                  font=('TkFixedFont', 9)
                  ).pack(anchor='w', padx=8, pady=(4, 6))

        btn_frame = ttk.LabelFrame(parent, text='Actions')
        btn_frame.pack(fill='x', padx=8, pady=4)
        row = ttk.Frame(btn_frame); row.pack(fill='x', padx=4, pady=4)
        # PRIMARY: this tab's own CALIBRATE button.
        tk.Button(row, text='CALIBRATE POSITION',
                  bg='#226699', fg='white',
                  font=('TkDefaultFont', 11, 'bold'),
                  width=22, height=2,
                  command=self._on_calibrate
                  ).pack(side='left', padx=4)
        tk.Button(row, text='Enter manual mode', bg='#3366aa', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_enter_manual_calibrate
                  ).pack(side='left', padx=4)
        tk.Button(row, text='Reset to 0', bg='#aa6633', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_calibrate_reset
                  ).pack(side='left', padx=4)
        # Pop-out the calibration tabs in a separate window (same domain).
        if not self.calib_only:
            tk.Button(row, text='Open in separate window', bg='#555577',
                      fg='white', font=('TkDefaultFont', 10, 'bold'),
                      command=lambda: self._spawn_calibration_window('position')
                      ).pack(side='left', padx=4)
        # Live calibration preview (vendor parity): the tag detection +
        # projected workspace rectangle, exactly what the web tool showed.
        self._make_image_preview(parent, '/calibration/image_result',
                                 'Calibration view (live)')

    def _build_depth_tab(self, parent, current):
        """Round 15: Depth tab - alignment status + plane fit + depth
        heatmap toggle. The big CALIBRATE DEPTH (plane refit) button
        re-fits the table plane via vendor SearchPlane from the
        latest depth frame."""
        header = ttk.LabelFrame(parent, text='Depth calibration')
        header.pack(fill='x', padx=8, pady=(8, 4))
        ttk.Label(header, foreground='#444', wraplength=720,
                  justify='left',
                  text=(
                      'DEPTH CALIBRATION (table plane fit + RGB alignment)\n'
                      '• RANSAC plane fit on the latest depth frame '
                      '(vendor SearchPlane). Persists [a, b, c, d] to '
                      'transform.yaml.\n'
                      '• Above-plane height gating: detections gate on '
                      '"how far above the table" instead of absolute '
                      'depth - hands and reflections are filtered.\n'
                      '• Depth->color TF lookup at boot aligns the depth '
                      'camera to RGB without needing the depth_to_color '
                      'topic.\n'
                      '\n'
                      'TURN ON "Show depth heatmap" to SEE whether the '
                      'depth camera is reading the cubes - they appear as '
                      'warmer patches inside the workspace overlay.'
                  )).pack(anchor='w', padx=8, pady=(2, 6))

        knob_frame = ttk.LabelFrame(parent, text='Depth tunables (live)')
        knob_frame.pack(fill='x', padx=8, pady=4)
        toggle_row = ttk.Frame(knob_frame); toggle_row.pack(fill='x', padx=4)
        for bname in ('use_depth_for_z', 'overlay_depth_view'):
            self._add_bool(toggle_row, bname,
                           bool(current.get(bname, False)))
        for name, lo, hi, res in CALIB_DEPTH_FLOAT_PARAMS:
            self._add_float(knob_frame, name, lo, hi, res,
                            float(current.get(name, lo)))
        for name, lo, hi, res in CALIB_DEPTH_INT_PARAMS:
            self._add_int(knob_frame, name, lo, hi, res,
                          int(current.get(name, lo)))

        # Status panel - depth-only fields.
        status_frame = ttk.LabelFrame(parent,
                                      text='Depth status (live)')
        status_frame.pack(fill='x', padx=8, pady=4)
        self.depth_status_var = tk.StringVar(value='(awaiting heartbeat...)')
        ttk.Label(status_frame, textvariable=self.depth_status_var,
                  foreground='#222', wraplength=720, justify='left',
                  font=('TkFixedFont', 9)
                  ).pack(anchor='w', padx=8, pady=(4, 6))

        btn_frame = ttk.LabelFrame(parent, text='Actions')
        btn_frame.pack(fill='x', padx=8, pady=4)
        row = ttk.Frame(btn_frame); row.pack(fill='x', padx=4, pady=4)
        # PRIMARY: this tab's own CALIBRATE button (plane refit).
        tk.Button(row, text='CALIBRATE DEPTH (plane refit)',
                  bg='#226666', fg='white',
                  font=('TkDefaultFont', 11, 'bold'),
                  width=28, height=2,
                  command=self._on_plane_refit
                  ).pack(side='left', padx=4)
        if not self.calib_only:
            tk.Button(row, text='Open in separate window', bg='#555577',
                      fg='white', font=('TkDefaultFont', 10, 'bold'),
                      command=lambda: self._spawn_calibration_window('depth')
                      ).pack(side='left', padx=4)

    def _on_push_logs(self):
        """Round 12 Z2 / Round 13 R13.1: run tools/push_logs.sh in a
        worker thread so the latest session logs land in the repo's
        logs/ folder + GitHub. Repo location is resolved from THIS
        file (symlink-aware) so the installer's clone location
        (~/jetarm_v5_src) is found automatically."""
        def _find_repo():
            env = os.environ.get('JETARM_V5_REPO')
            if env and os.path.isdir(os.path.join(env, '.git')):
                return env
            # tune_uiv5.py is symlinked into ros2_ws/src/app/app/ -
            # realpath() resolves it back to the real checkout.
            d = os.path.dirname(os.path.realpath(__file__))
            while d and d != '/':
                if os.path.isdir(os.path.join(d, '.git')):
                    return d
                d = os.path.dirname(d)
            for p in (os.path.expanduser('~/jetarm_v5_src'),
                      os.path.expanduser('~/new-repo2')):
                if os.path.isdir(os.path.join(p, '.git')):
                    return p
            return None

        def go():
            try:
                repo = _find_repo()
                if repo is None:
                    self._set_status('PUSH LOGS: no repo found '
                                     '(set JETARM_V5_REPO env)', '#aa6633')
                    _ui_log('push_logs', 'no repo found')
                    return
                script = os.path.join(repo, 'tools', 'push_logs.sh')
                if not os.path.isfile(script):
                    self._set_status(
                        f'PUSH LOGS: tools/push_logs.sh missing in {repo}',
                        '#aa6633')
                    _ui_log('push_logs', f'script missing in {repo}')
                    return
                self._set_status('PUSH LOGS: running...', '#3366aa')
                _ui_log('push_logs', f'running {script} REPO={repo}')
                env = os.environ.copy()
                env['REPO'] = repo
                res = subprocess.run(['bash', script], env=env,
                                     capture_output=True, text=True,
                                     timeout=120)
                # Round 14 DD.3: distinct rc codes from push_logs.sh:
                # 0 = pushed, 2 = no logs found, 3 = nothing new to
                # commit, 4 = git push failed (auth/network).
                rc = res.returncode
                if rc == 0:
                    self._set_status('PUSH LOGS: pushed to jetarm-logs branch',
                                     '#3366aa')
                    _ui_log('push_logs', 'OK')
                elif rc == 2:
                    self._set_status('PUSH LOGS: no logs found yet '
                                     '(start the node first)',
                                     '#aa6633')
                    _ui_log('push_logs', 'no logs found')
                elif rc == 3:
                    self._set_status('PUSH LOGS: nothing new to commit',
                                     '#666666')
                    _ui_log('push_logs', 'nothing new to commit')
                elif rc == 4:
                    tail = (res.stderr or '')[-300:]
                    self._set_status(
                        f'PUSH LOGS: git push failed - {tail.strip()[:80]}',
                        '#aa6633')
                    _ui_log('push_logs', f'push failed: {tail}')
                else:
                    self._set_status(
                        f'PUSH LOGS: rc={rc} - {(res.stderr or "")[-80:]}',
                        '#aa6633')
                    _ui_log('push_logs', f'rc={rc} '
                                          f'stderr={(res.stderr or "")[-500:]}')
            except Exception as e:
                self._set_status('PUSH LOGS: error', '#aa6633')
                _ui_log('push_logs', 'crashed', exc=e)
        threading.Thread(target=go, daemon=True).start()

    def _on_plane_refit(self):
        """Round 12 Y6 / Round 17 PP.4: call the node's depth_plane_refit
        service and surface the response.message (specific failure reason)
        instead of an opaque 'failed'."""
        def go():
            try:
                ok, msg = self.client._trigger_with_msg(
                    self.client.depth_plane_refit_cli)
                if ok:
                    self._set_status('Plane refit OK', '#3366aa')
                    _ui_log('plane_refit', 'OK')
                else:
                    reason = msg or 'unknown'
                    self._set_status(
                        f'PLANE REFIT FAILED: {reason}', '#aa6633')
                    _ui_log('plane_refit', f'failed: {reason}')
            except Exception as e:
                _ui_log('plane_refit', 'crashed', exc=e)
                self._set_status('Plane refit error', '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    # ---------------------------------------------------------------- Color tab
    # Round 14 AA.5: vendor lab_manager.py one-for-one. 6 LAB sliders +
    # mask preview + Enter/Exit/Stash/Save/Reload/Reset buttons.

    DEFAULT_LAB_RANGES = {
        'red':    {'min': [9, 146, 146],   'max': [160, 187, 207]},
        'green':  {'min': [72, 49, 128],   'max': [203, 113, 197]},
        'blue':   {'min': [11, 115, 46],   'max': [172, 178, 117]},
        'black':  {'min': [0, 30, 92],     'max': [86, 187, 196]},
        'white':  {'min': [111, 100, 100], 'max': [255, 155, 155]},
        'yellow': {'min': [195, 110, 146], 'max': [255, 177, 184]},
        'tennis': {'min': [31, 66, 178],   'max': [210, 159, 255]},
    }

    def _build_color_tab(self, parent, current):
        """LAB color calibration (vendor lab_manager.py one-for-one).
        Sliders push live ChangeRange requests so the published mask
        updates in real time."""
        header = ttk.LabelFrame(parent, text='How LAB color calibration works')
        header.pack(fill='x', padx=8, pady=(8, 4))
        ttk.Label(header, foreground='#444', wraplength=720,
                  justify='left',
                  text=(
                      'LAB COLOR THRESHOLD (drives vendor lab_manager_node)\n'
                      '* Click ENTER to start publishing a live mask on '
                      '/lab_manager/image_result.\n'
                      '* Pick a color (red / green / blue / etc.).\n'
                      '* Move the L / A / B min/max sliders - mask updates '
                      'in real time. Aim for a clean mask of just the '
                      'cube of that color.\n'
                      '* STASH saves the current sliders to the selected '
                      'color. SAVE writes lab_config.yaml to disk.\n'
                      '* Same yaml schema vendor sorting reads, so changes '
                      'are picked up by every other Hiwonder app too.\n'
                      '* RELOAD pulls the saved range back from disk for '
                      'the current color (handy for "I broke it - revert").'
                  )).pack(anchor='w', padx=8, pady=(2, 6))

        # Active color + status panel.
        top = ttk.LabelFrame(parent, text='Active color + status')
        top.pack(fill='x', padx=8, pady=4)
        row = ttk.Frame(top); row.pack(fill='x', padx=8, pady=4)
        ttk.Label(row, text='Color:').pack(side='left')
        self.color_active_var = tk.StringVar(value='red')
        self.color_dropdown = ttk.Combobox(
            row, textvariable=self.color_active_var, width=12,
            values=list(self.DEFAULT_LAB_RANGES.keys()), state='readonly')
        self.color_dropdown.pack(side='left', padx=6)
        self.color_dropdown.bind('<<ComboboxSelected>>',
                                 lambda e: self._on_color_dropdown_change())
        self.color_status_var = tk.StringVar(value='LAB: OFF')
        ttk.Label(row, textvariable=self.color_status_var,
                  foreground='#222', font=('TkFixedFont', 9)
                  ).pack(side='left', padx=12)

        # 6 sliders for L/A/B min + max (0..255).
        slid_frame = ttk.LabelFrame(parent,
                                    text='LAB thresholds (live)')
        slid_frame.pack(fill='x', padx=8, pady=4)
        self._lab_sliders = {}
        for label in ('L_min', 'A_min', 'B_min',
                      'L_max', 'A_max', 'B_max'):
            r = ttk.Frame(slid_frame); r.pack(fill='x', padx=6, pady=2)
            ttk.Label(r, text=label, width=8).pack(side='left')
            v = tk.IntVar(value=0 if label.endswith('_min') else 255)
            s = tk.Scale(r, from_=0, to=255, orient='horizontal',
                         variable=v, length=420, showvalue=True,
                         resolution=1,
                         command=lambda *_: self._lab_push_live_change())
            s.pack(side='left', fill='x', expand=True)
            self._lab_sliders[label] = v

        # Buttons.
        btn_frame = ttk.LabelFrame(parent, text='Actions')
        btn_frame.pack(fill='x', padx=8, pady=4)
        b_row = ttk.Frame(btn_frame); b_row.pack(fill='x', padx=4, pady=4)
        tk.Button(b_row, text='ENTER (start preview)', bg='#3366aa',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_color_enter
                  ).pack(side='left', padx=4)
        tk.Button(b_row, text='EXIT', bg='#666666', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_color_exit
                  ).pack(side='left', padx=4)
        tk.Button(b_row, text='STASH to color', bg='#226699', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_color_stash
                  ).pack(side='left', padx=4)
        tk.Button(b_row, text='SAVE to disk', bg='#2e8b57', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_color_save
                  ).pack(side='left', padx=4)
        tk.Button(b_row, text='Reload from disk', bg='#666666',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_color_reload
                  ).pack(side='left', padx=4)
        tk.Button(b_row, text='Reset color to default', bg='#aa6633',
                  fg='white', font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_color_reset
                  ).pack(side='left', padx=4)
        if not self.calib_only:
            tk.Button(b_row, text='Open in separate window', bg='#555577',
                      fg='white', font=('TkDefaultFont', 10, 'bold'),
                      command=lambda: self._spawn_calibration_window('color')
                      ).pack(side='left', padx=4)

        # Live LAB mask preview (vendor parity): the exact mono8 mask the
        # vendor web tool showed, straight from /lab_manager/image_result.
        self._make_image_preview(parent, '/lab_manager/image_result',
                                 'LAB mask (live - press ENTER first)',
                                 mono=True)

        # Pull starting values for the dropdown's default color.
        self.root.after(500, self._on_color_dropdown_change)

    def _on_color_dropdown_change(self):
        """Load saved range for the newly-selected color into sliders.
        Push it as the live mask range so the preview matches."""
        color = self.color_active_var.get()
        def go():
            rng = self.client.lab_get_range(color)
            if rng is None:
                # fall back to defaults so sliders still get something
                d = self.DEFAULT_LAB_RANGES.get(
                    color, {'min': [0, 0, 0], 'max': [255, 255, 255]})
                rng = (d['min'], d['max'])
            mn, mx = rng
            self.root.after(0,
                            lambda: self._lab_set_sliders(mn, mx))
            self.client.lab_change_range(mn, mx)
            _ui_log('color', f'loaded {color}: min={mn} max={mx}')
        threading.Thread(target=go, daemon=True).start()

    def _lab_set_sliders(self, mn, mx):
        for axis, val in zip(('L_min', 'A_min', 'B_min'), mn):
            self._lab_sliders[axis].set(int(val))
        for axis, val in zip(('L_max', 'A_max', 'B_max'), mx):
            self._lab_sliders[axis].set(int(val))

    def _lab_collect_sliders(self):
        mn = [self._lab_sliders[a].get() for a in ('L_min', 'A_min', 'B_min')]
        mx = [self._lab_sliders[a].get() for a in ('L_max', 'A_max', 'B_max')]
        # Clamp min<=max so cv2.inRange never inverts.
        for i in range(3):
            if mn[i] > mx[i]:
                mn[i], mx[i] = mx[i], mn[i]
        return mn, mx

    def _lab_push_live_change(self):
        """Debounced 120 ms; sends ChangeRange so the published mask
        updates in real time as sliders move."""
        if getattr(self, '_lab_pending_after', None) is not None:
            try:
                self.root.after_cancel(self._lab_pending_after)
            except Exception:
                pass
        self._lab_pending_after = self.root.after(
            120, self._lab_push_live_change_now)

    def _lab_push_live_change_now(self):
        self._lab_pending_after = None
        mn, mx = self._lab_collect_sliders()
        def go():
            self.client.lab_change_range(mn, mx)
        threading.Thread(target=go, daemon=True).start()

    def _on_color_enter(self):
        # Round 17 OO.3: capture the vendor's response message so failures
        # carry their real reason instead of a flat "enter failed".
        def go():
            ok, msg = self.client.lab_enter()
            if ok:
                self.color_status_var.set('LAB: ON (mask publishing)')
                self._set_status('LAB: enter OK', '#3366aa')
                _ui_log('color', f'enter OK ({msg})')
            else:
                reason = msg or 'no detail'
                self.color_status_var.set(f'LAB: enter failed - {reason}')
                self._set_status(f'LAB: enter failed - {reason}', '#aa6633')
                _ui_log('color', f'enter failed: {reason}')
        threading.Thread(target=go, daemon=True).start()

    def _on_color_exit(self):
        def go():
            ok, msg = self.client.lab_exit()
            self.color_status_var.set(
                'LAB: OFF' if ok else f'LAB: exit failed - {msg or "no detail"}')
            self._set_status('LAB: exit OK' if ok else f'LAB: exit failed - {msg}',
                             '#3366aa' if ok else '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    def _on_color_stash(self):
        color = self.color_active_var.get()
        mn, mx = self._lab_collect_sliders()
        def go():
            self.client.lab_change_range(mn, mx)
            ok = self.client.lab_stash_range(color)
            self._set_status(
                f'LAB: stashed -> {color}' if ok else
                f'LAB: stash {color} failed',
                '#3366aa' if ok else '#aa6633')
            _ui_log('color', f'stash {color}: ok={ok}')
        threading.Thread(target=go, daemon=True).start()

    def _on_color_save(self):
        def go():
            ok, msg = self.client.lab_save()
            self._set_status(
                'LAB: saved lab_config.yaml' if ok
                else f'LAB: save failed - {msg or "no detail"}',
                '#2e8b57' if ok else '#aa6633')
            _ui_log('color', f'save: ok={ok} msg={msg}')
        threading.Thread(target=go, daemon=True).start()

    def _on_color_reload(self):
        self._on_color_dropdown_change()

    def _on_color_reset(self):
        color = self.color_active_var.get()
        d = self.DEFAULT_LAB_RANGES.get(color)
        if d is None:
            return
        self._lab_set_sliders(d['min'], d['max'])
        self._lab_push_live_change_now()
        self._set_status(f'LAB: {color} reset to default', '#666666')

    def _on_enter_manual_calibrate(self):
        def go():
            try:
                self.client.set_value('calibrate_overlay_mode', 'manual')
                self._set_status('Calibrate: manual overlay ON', '#3366aa')
            except Exception:
                messagebox.showerror('Overlay',
                                     'Could not enable manual overlay mode.')
        threading.Thread(target=go, daemon=True).start()

    def _on_calibrate_reset(self):
        """Reset the Calibrate-tab sliders to their no-op defaults (live;
        not yet persisted). Offsets back to 0, scale to 1.0, workspace size
        back to 0.30 m so the overlay rectangle reappears at a reasonable
        size after a user has scaled it to nothing."""
        setters = getattr(self, '_param_setters', {})
        for name, default in (('grip_offset_x', 0.0),
                              ('grip_offset_y', 0.0),
                              ('workspace_scale', 1.0),
                              ('workspace_size_x', 0.30),
                              ('workspace_size_y', 0.30)):
            if name in setters:
                try:
                    setters[name](default)
                except Exception:
                    pass
            try:
                self.client.set_value(name, default)
            except Exception:
                pass

    def _detect_save_extra(self):
        """Engine path + task + class filter for the Detection tab's Save & Apply."""
        extra = {}
        try:
            extra['engine_path'] = self._model_engine_entry.get().strip()
        except Exception:
            pass
        try:
            extra['engine_task'] = (self._engine_task_var.get() or 'auto').strip()
        except Exception:
            pass
        if self._model_class_vars:
            sel = [self._model_class_id_map[n]
                   for n, v in self._model_class_vars.items() if v.get()]
            all_sel = len(sel) == len(self._model_class_vars)
            # Empty list = "all classes" (matches the node semantics).
            extra['yolo_enabled_classes'] = json.dumps([] if all_sel else sel)
        return extra

    def _on_apply_engine(self):
        """Commit the engine path in the entry and switch the running model
        immediately (no full Save needed)."""
        path = self._model_engine_entry.get().strip()
        if not path:
            messagebox.showerror('No engine', 'Pick or type an engine path first.')
            return
        self._set_status('SWITCHING ENGINE...', '#aa6633')
        def go():
            ok = self.client.call_load_engine(path)
            if ok:
                self._active_engine = path
                self._set_status('ENGINE SWITCHING', '#3366aa')
            else:
                messagebox.showerror(
                    'Switch failed',
                    'load_engine rejected (file missing on the Jetson?).')
                self._set_status('ENGINE SWITCH FAILED', '#cc3333')
        threading.Thread(target=go, daemon=True).start()

    def _set_all_classes(self, value):
        for var in self._model_class_vars.values():
            var.set(bool(value))
        self._mark_model_dirty()

    def _invert_classes(self):
        for var in self._model_class_vars.values():
            var.set(not var.get())
        self._mark_model_dirty()

    def _apply_class_filter_text(self):
        q = self._class_filter_entry.get().strip().lower()
        for frame, name in self._model_class_widgets:
            if q in name.lower():
                frame.pack(anchor='w', padx=4, pady=1)
            else:
                frame.pack_forget()

    def _refresh_class_filter(self, class_names):
        """Status-poll hook. Rebuilds the class grid when model.names
        changes - but NEVER while the user has unsaved edits, and preserves
        existing tick state for classes that survive a model change."""
        if not class_names:
            return
        if getattr(self, '_model_dirty', False):
            return  # don't wipe a half-made selection during a hot-swap poll
        items = sorted(((int(k), v) for k, v in class_names.items()),
                       key=lambda x: x[0])
        new_names = [v for _, v in items]
        if new_names == list(self._model_class_vars.keys()):
            return  # no change
        # Snapshot current ticks so surviving classes keep their state.
        prev = {n: v.get() for n, v in self._model_class_vars.items()}
        for w in self._model_classes_inner.winfo_children():
            w.destroy()
        self._model_class_vars.clear()
        self._model_class_id_map.clear()
        self._model_class_widgets = []
        # Seed from persisted yolo_enabled_classes for classes we haven't
        # seen before.
        # v5.1 FIX (A6d): the persisted list used to be fetched with a BLOCKING
        # self.client.get_values(...) call right here - but this runs on the Tk
        # poll thread (_poll_node_status via root.after), so a slow/unreachable
        # node froze the UI for up to 2s whenever class names changed. Fetch it
        # ONCE in a worker thread and cache it; this rebuild uses the cache, or
        # the all-on default until the cache is warm (next model-name change
        # picks it up). Same semantics as before: empty/None -> all classes on.
        cached = getattr(self, '_persisted_enabled_cache', None)
        if cached is None and not getattr(self, '_persist_fetch_inflight', False):
            self._persist_fetch_inflight = True
            def _fetch_persisted():
                try:
                    raw = (self.client.get_values(['yolo_enabled_classes'])
                           .get('yolo_enabled_classes', '[]') or '[]')
                    self._persisted_enabled_cache = json.loads(raw)
                except Exception:
                    self._persisted_enabled_cache = []
                finally:
                    self._persist_fetch_inflight = False
            threading.Thread(target=_fetch_persisted, daemon=True).start()
        try:
            persisted_set = set(int(x) for x in cached) if cached else None
        except Exception:
            persisted_set = None
        for cid, name in items:
            if name in prev:
                init = prev[name]
            elif persisted_set is not None:
                init = cid in persisted_set
            else:
                init = True
            var = tk.BooleanVar(value=init)
            self._model_class_vars[name] = var
            self._model_class_id_map[name] = cid
            row = ttk.Frame(self._model_classes_inner)
            row.pack(anchor='w', padx=4, pady=1)
            ttk.Checkbutton(row, text=f'{cid}: {name}', variable=var,
                            command=self._mark_model_dirty).pack(side='left')
            self._model_class_widgets.append((row, name))

    def _on_reset_model_defaults(self):
        """Reset the live YOLO knobs (conf / IoU / max-det / inference-Hz) to
        factory defaults and clear the class filter. Applies immediately;
        engine path is left alone. Persist with the Detection Save & Apply."""
        if not messagebox.askyesno(
                'Reset knobs to defaults',
                'Restore conf / IoU / max-det / inference-Hz to factory '
                'defaults and clear the class filter? Engine path is '
                'unchanged. Press the Detection "Save & Apply" to persist.'):
            return
        setters = getattr(self, '_param_setters', {})
        for name, val in MODEL_DEFAULTS.items():
            fn = setters.get(name)
            if fn is not None:
                fn(val)                       # update the slider/entry display
            self.client.set_value(name, val)  # apply live
        for var in self._model_class_vars.values():
            var.set(False)
        self._mark_model_dirty()
        self._set_status('KNOBS RESET (Save & Apply to persist)', '#aa6633')

    def _on_reload_engine(self):
        """Commit the engine path currently in the entry (so the picker
        selection actually takes effect) and re-init the model from it.
        Fixes "select a new engine + reload doesn't switch"."""
        if not messagebox.askyesno(
                'Reload engine',
                'Re-initialise the YOLO model from the engine path shown? '
                'Inference will pause briefly.'):
            return
        path = self._model_engine_entry.get().strip()
        self._set_status('ENGINE RELOADING...', '#aa6633')
        def go():
            # Commit the selected path first so reload targets the intended
            # engine, not the previously-loaded one.
            if path and path != self._active_engine:
                if self.client.call_load_engine(path):
                    self._active_engine = path
            ok = self.client.call_reload_engine()
            if ok:
                self._set_status('ENGINE RELOADED', '#3366aa')
            else:
                messagebox.showerror(
                    'Reload failed',
                    'reload_engine service rejected (calibration in '
                    'progress, or a swap is already pending).')
                self._set_status('RELOAD FAILED', '#cc3333')
        threading.Thread(target=go, daemon=True).start()

    # ---- engine picker helpers (fill the entry; Apply/Reload commit + swap) ----

    def _refresh_engine_list(self, current_path):
        self.engine_listbox.delete(0, tk.END)
        if not DEFAULT_ENGINES_DIR.exists():
            self.engine_listbox.insert(tk.END, f'(dir not found: {DEFAULT_ENGINES_DIR})')
            return
        # List both TensorRT engines and PyTorch weights.
        engines = sorted(DEFAULT_ENGINES_DIR.glob('*.engine')) + \
            sorted(DEFAULT_ENGINES_DIR.glob('*.pt'))
        if not engines:
            self.engine_listbox.insert(tk.END, '(no .engine / .pt files)')
            return
        # The node publishes the active engine as a basename, so match on that
        # too - that way [active] is correct right after a swap.
        active_base = os.path.basename(current_path or '')
        for p in engines:
            marker = ' [active]' if p.name == active_base else ''
            self.engine_listbox.insert(tk.END, f'{p.name}{marker}')

    def _set_engine_buffer(self, path):
        # Just fill the path entry; "Apply engine" / "Reload engine" / the tab
        # Save & Apply are the explicit commit. (Don't mark the class-filter
        # dirty flag here - that would block the class grid from refreshing.)
        self._model_engine_entry.delete(0, 'end')
        self._model_engine_entry.insert(0, path)

    def _selected_engine_path(self):
        sel = self.engine_listbox.curselection()
        if not sel:
            return None
        text = self.engine_listbox.get(sel[0]).split(' [active]')[0]
        # v5.1 FIX (A6c): the listbox shows '(dir not found...)' / '(no .engine
        # / .pt files)' placeholders when no engines exist - don't turn one of
        # those into a bogus engine path.
        if not text or text.startswith('('):
            return None
        return str(DEFAULT_ENGINES_DIR / text)

    def _on_pick_selected_engine(self):
        path = self._selected_engine_path()
        if path:
            self._set_engine_buffer(path)

    def _on_pick_and_apply_engine(self):
        """Double-click handler: fill the entry from the selection and swap
        to it immediately (the old one-step model-switch flow)."""
        path = self._selected_engine_path()
        if path:
            self._set_engine_buffer(path)
            self._on_apply_engine()

    def _on_browse_engine(self):
        path = filedialog.askopenfilename(
            title='Pick model file', initialdir=str(DEFAULT_ENGINES_DIR),
            filetypes=[('TensorRT engine', '*.engine'),
                       ('PyTorch', '*.pt'), ('All files', '*.*')])
        if path:
            self._set_engine_buffer(path)

    # ---- Places tab (per-class targets: place position + grip strength) ----

    # ------------------------------------------------------------------ bin teach
    def _build_teach_tab(self, parent, current_places_json='{}'):
        """Manual bin teaching. Jog the arm to each physical bin (joint-by-joint
        or world XYZ), read the live coordinate, and SAVE it per class so the
        cube lands exactly there. Also teach the workspace centre/edges to
        re-anchor the world map. All motion goes through the node ~/teach
        service (refused while sorting/calibrating)."""
        try:
            self._teach_places = json.loads(current_places_json or '{}')
            if not isinstance(self._teach_places, dict):
                self._teach_places = {}
        except Exception:
            self._teach_places = {}
        self._teach_bins_rows = {}
        try:
            o = self.client.get_values(['grip_offset_x', 'grip_offset_y', 'grip_offset_z'])
        except Exception:
            o = {}
        self._pick_off = {
            'grip_offset_x': float(o.get('grip_offset_x', 0.0) or 0.0),
            'grip_offset_y': float(o.get('grip_offset_y', 0.0) or 0.0),
            'grip_offset_z': float(o.get('grip_offset_z', 0.0) or 0.0)}
        JS = ('TkDefaultFont', 9)
        ttk.Label(parent, foreground='#226666', wraplength=720, justify='left',
                  text='Move the arm to a bin, read its coordinate, then Save it '
                       'for a class. A SAVED bin is just the drop LOCATION - the '
                       'arm works out its own approach and drops exactly there '
                       '(taught bins skip the vision correction). STOP sorting '
                       'first; the arm MOVES. Joint jog never needs IK, so use it '
                       'to reach bins the world jog can’t.'
                  ).pack(anchor='w', padx=8, pady=(8, 4))

        # ---- live readout ------------------------------------------------
        ro = ttk.LabelFrame(parent, text='Live readout (from the arm)')
        ro.pack(fill='x', padx=8, pady=4)
        self.teach_pose_var = tk.StringVar(value='world (x, y, z): --')
        self.teach_joints_var = tk.StringVar(value='servos 1-5: --')
        self.teach_center_var = tk.StringVar(value='workspace: centre not staged')
        self.teach_msg_var = tk.StringVar(value='status: idle')
        ttk.Label(ro, textvariable=self.teach_pose_var, font=('TkDefaultFont', 10, 'bold'),
                  foreground='#2e6e2e').pack(anchor='w', padx=8, pady=(6, 0))
        ttk.Label(ro, textvariable=self.teach_joints_var, font=JS).pack(anchor='w', padx=8)
        ttk.Label(ro, textvariable=self.teach_center_var, font=JS,
                  foreground='#555').pack(anchor='w', padx=8)
        ttk.Label(ro, textvariable=self.teach_msg_var, font=JS, foreground='#996600',
                  wraplength=720, justify='left').pack(anchor='w', padx=8, pady=(0, 4))
        ttk.Button(ro, text='Refresh readout',
                   command=lambda: self._teach_send({'action': 'read'})
                   ).pack(anchor='w', padx=8, pady=(0, 6))

        # ---- step sizes --------------------------------------------------
        steps = ttk.LabelFrame(parent, text='Step size')
        steps.pack(fill='x', padx=8, pady=4)
        row = ttk.Frame(steps); row.pack(fill='x', padx=8, pady=6)
        ttk.Label(row, text='world step (m):').pack(side='left')
        self.teach_world_step = ttk.Entry(row, width=8, justify='right')
        self.teach_world_step.insert(0, '0.01'); self.teach_world_step.pack(side='left', padx=(2, 16))
        ttk.Label(row, text='joint step (pulses):').pack(side='left')
        self.teach_joint_step = ttk.Entry(row, width=8, justify='right')
        self.teach_joint_step.insert(0, '10'); self.teach_joint_step.pack(side='left', padx=2)

        # ---- joint-by-joint jog -----------------------------------------
        jf = ttk.LabelFrame(parent, text='Joint jog (one servo at a time - always reachable)')
        jf.pack(fill='x', padx=8, pady=4)
        ttk.Label(jf, foreground='#996600', wraplength=720, justify='left',
                  text='Press HOME first (below) — this arm can’t report its '
                       'servo angles, so jogs are tracked from the Home pose. '
                       'After Home, each ± moves that servo by the joint step.'
                  ).pack(anchor='w', padx=8, pady=(6, 2))
        joint_labels = [(1, 'base rotate'), (2, 'shoulder'), (3, 'elbow'),
                        (4, 'wrist pitch'), (5, 'wrist rotate')]
        for jid, hint in joint_labels:
            r = ttk.Frame(jf); r.pack(fill='x', padx=8, pady=2)
            ttk.Label(r, text=f'J{jid}  {hint}', width=18).pack(side='left')
            ttk.Button(r, text='–', width=4,
                       command=lambda j=jid: self._on_teach_joint(j, -1)).pack(side='left', padx=2)
            ttk.Button(r, text='+', width=4,
                       command=lambda j=jid: self._on_teach_joint(j, +1)).pack(side='left', padx=2)
        ttk.Button(jf, text='HOME (sync joints + go to home pose)',
                   command=lambda: self._teach_send({'action': 'home'})
                   ).pack(anchor='w', padx=8, pady=(4, 6))

        # ---- world XYZ jog ----------------------------------------------
        wf = ttk.LabelFrame(parent, text='World jog (straight-line X/Y/Z - uses IK)')
        wf.pack(fill='x', padx=8, pady=4)
        for axis, hint in (('x', 'left / right'), ('y', 'near / far'), ('z', 'down / up')):
            r = ttk.Frame(wf); r.pack(fill='x', padx=8, pady=2)
            ttk.Label(r, text=f'{axis.upper()}  {hint}', width=18).pack(side='left')
            ttk.Button(r, text='–', width=4,
                       command=lambda a=axis: self._on_teach_world(a, -1)).pack(side='left', padx=2)
            ttk.Button(r, text='+', width=4,
                       command=lambda a=axis: self._on_teach_world(a, +1)).pack(side='left', padx=2)
        ttk.Button(wf, text='Home (safe centre pose)',
                   command=lambda: self._teach_send({'action': 'home'})
                   ).pack(anchor='w', padx=8, pady=(4, 6))

        # ---- save current coordinate as a class bin ---------------------
        sf = ttk.LabelFrame(parent, text='Save the readout as a bin')
        sf.pack(fill='x', padx=8, pady=4)
        r = ttk.Frame(sf); r.pack(fill='x', padx=8, pady=6)
        ttk.Label(r, text='class:').pack(side='left')
        self.teach_class_var = tk.StringVar()
        self.teach_class_combo = ttk.Combobox(r, textvariable=self.teach_class_var,
                                               width=18, state='normal')
        self.teach_class_combo.pack(side='left', padx=4)
        ttk.Button(r, text='Go to bin', command=self._on_teach_goto).pack(side='left', padx=4)
        ttk.Button(r, text='Save here → bin', command=self._on_teach_save).pack(side='left', padx=4)
        self.teach_persist_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(sf, text='persist to default.yaml (survives relaunch)',
                        variable=self.teach_persist_var).pack(anchor='w', padx=8, pady=(0, 6))

        # ---- saved bins: jump to an existing bin to fine-tune ------------
        bf = ttk.LabelFrame(parent, text='Saved bins — Go, then fine-tune & re-save')
        bf.pack(fill='x', padx=8, pady=4)
        ttk.Label(bf, foreground='#555', wraplength=720, justify='left',
                  text='Press Go to drive the arm to a bin you already set, then '
                       'nudge with the jog buttons and Save again — no need to '
                       'hand-jog from scratch. "(default)" bins have no taught '
                       'coordinate yet; Go still drives to the built-in spot.'
                  ).pack(anchor='w', padx=8, pady=(6, 2))
        self._teach_bins_inner = ttk.Frame(bf)
        self._teach_bins_inner.pack(fill='x', padx=8, pady=4)
        ttk.Label(self._teach_bins_inner, foreground='#888',
                  text='(waiting for engine load...)').pack(anchor='w')

        # ---- workspace centre / edges -----------------------------------
        kf = ttk.LabelFrame(parent, text='Workspace map (centre + edges → world origin)')
        kf.pack(fill='x', padx=8, pady=4)
        ttk.Label(kf, foreground='#555', wraplength=720, justify='left',
                  text='Run AprilTag CALIBRATE first (it sets the camera geometry). '
                       'Then jog to the mat CENTRE and Set Centre; jog to each EDGE '
                       'and Add Edge; Save Workspace re-anchors the world origin to '
                       'your centre and sizes the overlay from the edges. The camera '
                       'calibration is left untouched.'
                  ).pack(anchor='w', padx=8, pady=(6, 2))
        r = ttk.Frame(kf); r.pack(fill='x', padx=8, pady=2)
        ttk.Button(r, text='Set centre',
                   command=lambda: self._teach_send({'action': 'set_center'})).pack(side='left', padx=3)
        ttk.Button(r, text='Add edge',
                   command=lambda: self._teach_send({'action': 'add_edge'})).pack(side='left', padx=3)
        ttk.Button(r, text='Clear',
                   command=lambda: self._teach_send({'action': 'clear_workspace'})).pack(side='left', padx=3)
        ttk.Button(r, text='Save workspace',
                   command=self._on_teach_save_workspace).pack(side='left', padx=3)
        self.teach_force_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(kf, text='Confirm re-anchor (apply even if far from current origin)',
                        variable=self.teach_force_var).pack(anchor='w', padx=8, pady=(2, 6))

        # ---- pick alignment: line up the grab with the camera -----------
        af = ttk.LabelFrame(parent, text='Pick alignment — line up the grab with what the camera sees')
        af.pack(fill='x', padx=8, pady=4)
        ttk.Label(af, foreground='#555', wraplength=720, justify='left',
                  text='If the arm grabs off-target, nudge the world offset here. '
                       '+X moves the grab further RIGHT, +Y further away, +Z lifts '
                       'it OFF the table. It applies to the NEXT pick — run sorting, '
                       'watch a grab, nudge, repeat, then Save. These are PICK '
                       'offsets (grab side), separate from the bins (place side). '
                       'e.g. a grab landing +7 cm right & too low: X about -0.07, '
                       'Z about +0.03.'
                  ).pack(anchor='w', padx=8, pady=(6, 2))
        self.pick_off_var = tk.StringVar(value='offset  x=+0.000  y=+0.000  z=+0.000 m')
        ttk.Label(af, textvariable=self.pick_off_var, font=('TkDefaultFont', 10, 'bold'),
                  foreground='#2e6e2e').pack(anchor='w', padx=8, pady=(0, 2))
        sr = ttk.Frame(af); sr.pack(fill='x', padx=8, pady=2)
        ttk.Label(sr, text='step (m):').pack(side='left')
        self.pick_off_step = ttk.Entry(sr, width=8, justify='right')
        self.pick_off_step.insert(0, '0.005'); self.pick_off_step.pack(side='left', padx=(2, 8))
        for axis, hint in (('x', 'left – / right +'), ('y', 'near – / far +'),
                           ('z', 'lower – / lift +')):
            r = ttk.Frame(af); r.pack(fill='x', padx=8, pady=2)
            ttk.Label(r, text=f'{axis.upper()}  {hint}', width=16).pack(side='left')
            ttk.Button(r, text='–', width=4,
                       command=lambda a=axis: self._on_pick_offset(a, -1)).pack(side='left', padx=2)
            ttk.Button(r, text='+', width=4,
                       command=lambda a=axis: self._on_pick_offset(a, +1)).pack(side='left', padx=2)
        br = ttk.Frame(af); br.pack(fill='x', padx=8, pady=(4, 6))
        ttk.Button(br, text='Save alignment',
                   command=self._on_pick_offset_save).pack(side='left', padx=3)
        ttk.Button(br, text='Zero',
                   command=self._on_pick_offset_zero).pack(side='left', padx=3)
        self._update_pick_off_label()

    def _pick_off_step(self):
        try:
            return abs(float(self.pick_off_step.get()))
        except Exception:
            return 0.005

    def _update_pick_off_label(self):
        o = self._pick_off
        self.pick_off_var.set('offset  x={:+.3f}  y={:+.3f}  z={:+.3f} m'.format(
            o['grip_offset_x'], o['grip_offset_y'], o['grip_offset_z']))

    def _on_pick_offset(self, axis, sign):
        key = {'x': 'grip_offset_x', 'y': 'grip_offset_y', 'z': 'grip_offset_z'}[axis]
        lim = 0.1 if axis == 'z' else 0.5
        newv = round(max(-lim, min(lim, self._pick_off[key] + sign * self._pick_off_step())), 4)
        self._pick_off[key] = newv
        self._update_pick_off_label()
        threading.Thread(target=lambda: self.client.set_value(key, float(newv)),
                         daemon=True).start()

    def _on_pick_offset_save(self):
        params = dict(self._pick_off)
        threading.Thread(target=lambda: self.client.apply_and_persist(params, True),
                         daemon=True).start()
        messagebox.showinfo('Saved', 'Pick alignment saved to default.yaml:\n  '
                            + '   '.join(f'{k.split("_")[-1]}={v:+.3f}'
                                         for k, v in params.items()))

    def _on_pick_offset_zero(self):
        for k in self._pick_off:
            self._pick_off[k] = 0.0
        self._update_pick_off_label()
        def go():
            for k in ('grip_offset_x', 'grip_offset_y', 'grip_offset_z'):
                self.client.set_value(k, 0.0)
        threading.Thread(target=go, daemon=True).start()

    def _teach_send(self, cmd, persist=False):
        """Fire a ~/teach command on a worker thread (never block Tk).
        Results arrive on the ~/teach_status topic and refresh the readout."""
        def go():
            try:
                self.client.call_teach(cmd, persist)
            except Exception:
                pass
        threading.Thread(target=go, daemon=True).start()

    def _teach_world_step(self):
        try:
            return abs(float(self.teach_world_step.get()))
        except Exception:
            return 0.01

    def _teach_joint_step(self):
        try:
            return abs(int(float(self.teach_joint_step.get())))
        except Exception:
            return 10

    def _on_teach_joint(self, jid, sign):
        self._teach_send({'action': 'joint_jog', 'joint': int(jid),
                          'delta': int(sign) * self._teach_joint_step()})

    def _on_teach_world(self, axis, sign):
        cmd = {'action': 'jog', 'dx': 0, 'dy': 0, 'dz': 0,
               'step': self._teach_world_step()}
        cmd['d' + axis] = int(sign)
        self._teach_send(cmd)

    def _on_teach_goto(self):
        cls = self.teach_class_var.get().strip()
        if not cls:
            messagebox.showinfo('Pick a class', 'Choose a class to go to its bin.')
            return
        self._teach_send({'action': 'goto', 'class': cls})

    def _on_teach_goto_class(self, cls):
        """Drive to a saved bin from the 'Saved bins' list, and mirror the class
        into the dropdown so the next 'Save here -> bin' re-saves THIS bin after
        you fine-tune it."""
        self.teach_class_var.set(cls)
        self._teach_send({'action': 'goto', 'class': cls})

    def _on_teach_save(self):
        cls = self.teach_class_var.get().strip()
        if not cls:
            messagebox.showinfo('Pick a class', 'Choose (or type) the class to save.')
            return
        if not messagebox.askyesno(
                'Save bin',
                f'Save the current arm coordinate as the "{cls}" bin?\n\n'
                f'Every "{cls}" detection will then be dropped here.'):
            return
        self._teach_send({'action': 'save', 'class': cls},
                         persist=bool(self.teach_persist_var.get()))

    def _on_teach_save_workspace(self):
        if not messagebox.askyesno(
                'Save workspace',
                'Re-anchor the world origin to the staged centre (and size the '
                'overlay from the staged edges)?\n\n'
                'This rewrites white_area_pose_world in transform.yaml. The '
                'AprilTag camera calibration is left untouched. Make sure the '
                'centre was taught at the true mat centre.'):
            return
        self._teach_send({'action': 'save_workspace',
                          'force': bool(self.teach_force_var.get())},
                         persist=bool(self.teach_persist_var.get()))

    def _build_places_tab(self, parent, current_places_json):
        """Per-class targets editor. Rows populate from model.names (~/status).
        Each row: place x/y/z + max grip strength. Per-row save AND save-all.
        place_positions and grasp_strength are live params (the user is
        explicitly clicking Save)."""
        try:
            self._places = json.loads(current_places_json or '{}')
            if not isinstance(self._places, dict):
                self._places = {}
        except Exception:
            self._places = {}
        try:
            self._grasp_strength = json.loads(
                self.client.get_values(['grasp_strength'])
                .get('grasp_strength', '{}') or '{}')
            if not isinstance(self._grasp_strength, dict):
                self._grasp_strength = {}
        except Exception:
            self._grasp_strength = {}
        ttk.Label(parent, foreground='#226666', wraplength=720,
                  justify='left',
                  text='Per-class targets. Set the world (x, y, z) drop point '
                       'and the MAX grip strength (close pulse cap, used by the '
                       'force-limited BETA grasp) for each detected class. Rows '
                       'auto-populate when the engine loads. CALIBRATE (top bar) '
                       'runs AprilTag calibration to keep IK/workspace accurate.'
                  ).pack(anchor='w', padx=8, pady=(8, 4))
        topbtn = ttk.Frame(parent); topbtn.pack(fill='x', padx=8, pady=2)
        ttk.Button(topbtn, text='Save all rows',
                   command=self._on_save_all_places).pack(side='left', padx=4)
        self._places_inner = ttk.Frame(parent)
        self._places_inner.pack(fill='both', expand=True, padx=8, pady=4)
        self._places_rows = {}  # class -> {'x','y','z','strength'} entries
        ttk.Label(self._places_inner, foreground='#888',
                  text='(waiting for engine load...)').pack(anchor='w')

    def _refresh_places(self, class_names):
        if not class_names:
            return
        names = sorted(class_names.values())
        if list(self._places_rows.keys()) == names:
            return
        for w in self._places_inner.winfo_children():
            w.destroy()
        self._places_rows = {}
        header = ttk.Frame(self._places_inner); header.pack(fill='x', pady=(0, 4))
        ttk.Label(header, text='class', width=16,
                  font=('TkDefaultFont', 9, 'bold')).pack(side='left')
        for col in ('x', 'y', 'z', 'grip'):
            ttk.Label(header, text=col, width=9,
                      font=('TkDefaultFont', 9, 'bold')).pack(side='left')
        for name in names:
            row = ttk.Frame(self._places_inner); row.pack(fill='x', pady=2)
            ttk.Label(row, text=name, width=16).pack(side='left')
            existing = self._places.get(name, [0.0, 0.2, 0.015])
            entries = {}
            for i, col in enumerate(('x', 'y', 'z')):
                e = ttk.Entry(row, width=9, justify='right')
                e.insert(0, f'{float(existing[i]):.3f}')
                e.pack(side='left', padx=2); entries[col] = e
            se = ttk.Entry(row, width=9, justify='right')
            se.insert(0, str(int(self._grasp_strength.get(name, 540))))
            se.pack(side='left', padx=2); entries['strength'] = se
            ttk.Button(row, text='Save',
                       command=lambda nm=name, ents=entries:
                           self._on_save_place(nm, ents)).pack(side='left', padx=6)
            ttk.Button(row, text='Test grip',
                       command=lambda nm=name, ents=entries:
                           self._on_test_grip(nm, ents)).pack(side='left', padx=2)
            self._places_rows[name] = entries

    def _parse_place_row(self, name, entries):
        x = float(entries['x'].get()); y = float(entries['y'].get())
        z = float(entries['z'].get()); s = int(float(entries['strength'].get()))
        return [x, y, z], s

    def _on_test_grip(self, name, entries):
        # Save this row's strength FIRST so the test uses the latest value
        # (the operator typically tweaks the number then clicks Test grip).
        try:
            _, strength = self._parse_place_row(name, entries)
            self._grasp_strength[name] = strength
        except ValueError:
            messagebox.showerror('Bad value', f'{name}: grip must be a number.')
            return
        if not messagebox.askyesno(
                'Test grip',
                f'Test grip for "{name}" (max strength {strength} pulses)?\n\n'
                f'The arm will move to a safe test pose, open the gripper, '
                f'and dwell so you can place the object between the jaws. '
                f'It then closes until contact (or the strength cap), holds '
                f'briefly, and releases.\n\nMake sure sorting is STOPPED.'):
            return
        def go():
            # Persist the strength so the test uses the saved value.
            self.client.set_value('grasp_strength', json.dumps(self._grasp_strength))
            ok = self.client.call_test_grip(name)
            if not ok:
                messagebox.showerror(
                    'Test grip refused',
                    f'Node rejected the test_grip call for "{name}". '
                    f'Make sure sorting is STOPPED, calibration is not '
                    f'running, and kinematics is up.')
        threading.Thread(target=go, daemon=True).start()

    def _on_save_place(self, name, entries):
        try:
            pos, strength = self._parse_place_row(name, entries)
        except ValueError:
            messagebox.showerror('Bad value', f'{name}: x/y/z and grip must be numbers.')
            return
        self._places[name] = pos
        self._grasp_strength[name] = strength
        def go():
            self.client.set_value('place_positions', json.dumps(self._places))
            self.client.set_value('grasp_strength', json.dumps(self._grasp_strength))
            self._set_status(f'TARGET {name} SAVED', '#3366aa')
        threading.Thread(target=go, daemon=True).start()

    def _on_save_all_places(self):
        bad = []
        for name, entries in self._places_rows.items():
            try:
                pos, strength = self._parse_place_row(name, entries)
                self._places[name] = pos
                self._grasp_strength[name] = strength
            except ValueError:
                bad.append(name)
        if bad:
            messagebox.showerror('Bad value',
                                 f'Skipped (non-numeric): {", ".join(bad)}')
        def go():
            self.client.set_value('place_positions', json.dumps(self._places))
            self.client.set_value('grasp_strength', json.dumps(self._grasp_strength))
            self._set_status('ALL TARGETS SAVED', '#3366aa')
        threading.Thread(target=go, daemon=True).start()

    # ---- Profiles tab ----

    def _build_profiles_tab(self, parent):
        ttk.Label(parent,
                  text=f'Profiles in {PROFILES_DIR}',
                  foreground='#444').pack(anchor='w', padx=8, pady=(8, 4))
        listbox_frame = ttk.Frame(parent); listbox_frame.pack(fill='both', expand=True,
                                                              padx=8, pady=4)
        self.profile_listbox = tk.Listbox(listbox_frame, height=8)
        self.profile_listbox.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(listbox_frame, orient='vertical',
                                  command=self.profile_listbox.yview)
        scrollbar.pack(side='right', fill='y')
        self.profile_listbox.configure(yscrollcommand=scrollbar.set)
        self._refresh_profile_list()

        btn_row = ttk.Frame(parent); btn_row.pack(fill='x', padx=8, pady=6)
        ttk.Button(btn_row, text='Refresh',
                   command=self._refresh_profile_list).pack(side='left', padx=4)
        ttk.Button(btn_row, text='Load selected',
                   command=self._on_load_selected_profile).pack(side='left', padx=4)

        ttk.Label(parent, text='Save current settings as profile:',
                  foreground='#444').pack(anchor='w', padx=8, pady=(8, 2))
        save_row = ttk.Frame(parent); save_row.pack(fill='x', padx=8)
        self.profile_entry = ttk.Entry(save_row)
        self.profile_entry.pack(side='left', fill='x', expand=True)
        ttk.Button(save_row, text='Save',
                   command=self._on_save_profile).pack(side='left', padx=4)

        ttk.Label(parent, foreground='#555',
                  text='Profiles are YAML files of every tunable. Loading one '
                       'pushes its values to the running node. SAVE AS DEFAULT '
                       '(top bar) makes the current state the boot profile.'
                  ).pack(anchor='w', padx=8, pady=(8, 4))

    def _refresh_profile_list(self):
        self.profile_listbox.delete(0, tk.END)
        if not PROFILES_DIR.exists():
            self.profile_listbox.insert(tk.END, '(no profiles dir yet)')
            return
        ys = sorted(PROFILES_DIR.glob('*.yaml'))
        if not ys:
            self.profile_listbox.insert(tk.END, '(no profiles yet)')
            return
        for p in ys:
            self.profile_listbox.insert(tk.END, p.name)

    def _on_load_selected_profile(self):
        sel = self.profile_listbox.curselection()
        if not sel: return
        name = self.profile_listbox.get(sel[0])
        def go():
            ok = self.client.call_load_profile(name)
            if ok:
                self._set_status('PROFILE LOADED', '#3366aa')
            else:
                messagebox.showerror('Load failed',
                                     f'Could not load profile {name}')
        threading.Thread(target=go, daemon=True).start()

    def _on_save_profile(self):
        name = self.profile_entry.get().strip()
        if not name:
            messagebox.showinfo('Name required', 'Enter a profile name first.')
            return
        if _slugify_preset(name) in RESERVED_PRESET_SLUGS:
            messagebox.showerror(
                'Reserved name',
                f'"{name}" is reserved (built-in preset or boot default). '
                'Pick a different name.')
            return
        def go():
            ok = self.client.call_save_profile(name)
            if ok:
                self._refresh_profile_list()
                self._refresh_preset_combo()
                self._set_status('PROFILE SAVED', '#3366aa')
        threading.Thread(target=go, daemon=True).start()

    # ---- namable custom presets (presets bar) ----

    def _custom_preset_names(self):
        """User preset files in PROFILES_DIR, excluding the boot default.yaml
        and the built-in speed presets (slow/medium/fast.yaml) - those have
        their own dedicated buttons (v5.1)."""
        if not PROFILES_DIR.exists():
            return []
        return [p.name for p in sorted(PROFILES_DIR.glob('*.yaml'))
                if _slugify_preset(p.name) not in RESERVED_PRESET_SLUGS]

    def _refresh_preset_combo(self):
        try:
            self._preset_combo['values'] = self._custom_preset_names()
        except Exception:
            pass

    def _on_save_preset(self):
        """Save ALL current settings as a namable custom preset. Never
        overwrites a built-in quick preset or the boot default."""
        name = self._preset_name_entry.get().strip()
        if not name:
            messagebox.showinfo('Name required', 'Type a preset name first.')
            return
        if _slugify_preset(name) in RESERVED_PRESET_SLUGS:
            messagebox.showerror(
                'Reserved name',
                f'"{name}" is reserved (built-in preset or boot default). '
                'Pick a different name for your custom preset.')
            return
        def go():
            ok = self.client.call_save_profile(name)
            if ok:
                self._refresh_preset_combo()
                self._refresh_profile_list()
                self._set_status(f'PRESET {name} SAVED', '#3366aa')
            else:
                messagebox.showerror('Save failed',
                                     f'Could not save preset {name}.')
        threading.Thread(target=go, daemon=True).start()

    def _on_load_preset(self):
        """Load a custom preset: applies every setting and swaps the engine
        if the preset names a different one."""
        name = self._preset_combo.get().strip()
        if not name:
            messagebox.showinfo('Pick a preset', 'Select a custom preset first.')
            return
        self._set_status(f'LOADING {name}...', '#aa6633')
        def go():
            ok = self.client.call_load_profile(name)
            if ok:
                self._set_status(f'PRESET {name} LOADED', '#3366aa')
            else:
                messagebox.showerror('Load failed',
                                     f'Could not load preset {name}.')
                self._set_status('LOAD FAILED', '#cc3333')
        threading.Thread(target=go, daemon=True).start()

    # ---- Control buttons ----

    def _set_status(self, text, color):
        try:
            self.status_var.set(text)
            self.status_label.configure(bg=color)
        except Exception:
            pass

    def _on_start(self):
        def go():
            ok_enter = self.client.call_enter()
            ok = self.client.call_enable_sorting(True)
            if ok and ok_enter:
                self._set_status('RUNNING', '#2e8b57')
            else:
                self._set_status('NODE NOT REACHABLE', '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    def _on_stop(self):
        def go():
            ok = self.client.call_enable_sorting(False)
            self._set_status('STOPPED' if ok else 'NODE NOT REACHABLE',
                             '#aa3333' if ok else '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    def _on_calibrate(self):
        # AprilTag calibration: STOPS sorting and MOVES the arm.
        # Round 12 X2: honest confirm text; X1: surface failure reason
        # from heartbeat last_calibrate.error so the user sees WHY a run
        # failed instead of a flat "CALIBRATE FAILED".
        if not messagebox.askyesno(
                'Run calibration?',
                'AprilTag calibration.\n\n'
                'Requires:\n'
                '  - ONE AprilTag (ID 1/2/3/100, 2.5 cm, tag36h11).\n'
                '  - Tag flat on the mat and fully in camera view.\n'
                '  - Workspace clear (the arm moves to its calibration pose).\n\n'
                'Calibration runs entirely through the vendor calibration_node '
                '(/calibration/*); v5 does not reimplement it.\n\n'
                'Continue?'):
            return
        def go():
            _ui_log('calibrate', 'CALIBRATE button pressed')
            self.client.call_enable_sorting(False)
            self._set_status('CALIBRATING...', '#3366aa')
            self.client.call_run_calibration()
            # Wait briefly for the heartbeat to refresh, then read the
            # most recent last_calibrate to determine real status + reason.
            time.sleep(2.0)
            for _ in range(20):
                st = getattr(self, '_last_status', None) or {}
                last = st.get('last_calibrate')
                if last and last.get('ts'):
                    if last.get('ok'):
                        src = last.get('source', '?')
                        self._set_status(f'CALIBRATED ({src})', '#aa3333')
                        _ui_log('calibrate', f'OK source={src}')
                    else:
                        reason = last.get('error') or 'unknown'
                        src = last.get('source', '?')
                        self._set_status(
                            f'CALIBRATE FAILED ({src}): {reason}', '#aa6633')
                        _ui_log('calibrate', f'FAILED source={src} reason={reason}')
                    return
                time.sleep(0.5)
            self._set_status('CALIBRATE TIMED OUT', '#aa6633')
            _ui_log('calibrate', 'timed out waiting for heartbeat')
        threading.Thread(target=go, daemon=True).start()

    def _on_toggle_camera_sub(self):
        # Pause/Resume the node's subscription to /depth_cam/rgb/image_raw.
        # This is OUR subscription only - the orbbec driver keeps publishing.
        # When paused: cam_fps -> 0, _raw_republish_tick still republishes
        # the last frame so a viewer doesn't go fully blank.
        def go():
            current = bool(self.client.get_values(['enable_camera_sub'])
                           .get('enable_camera_sub', True))
            self.client.set_value('enable_camera_sub', not current)
            new_state = not current
            self.cam_toggle_btn.configure(
                text='Resume camera' if not new_state else 'Pause camera',
                bg='#aa6600' if not new_state else '#996633')
        threading.Thread(target=go, daemon=True).start()

    def _on_toggle_inference(self):
        # Pause/Resume the InferenceWorker. Camera path keeps running and
        # publishing raw frames; we just don't burn GPU cycles on YOLO.
        def go():
            current = bool(self.client.get_values(['enable_inference'])
                           .get('enable_inference', True))
            self.client.set_value('enable_inference', not current)
            new_state = not current
            self.ai_toggle_btn.configure(
                text='Resume AI' if not new_state else 'Pause AI',
                bg='#aa3399' if not new_state else '#664488')
        threading.Thread(target=go, daemon=True).start()

    def _on_save_default(self):
        if not messagebox.askyesno(
                'Save ALL as default (boot)',
                'Persist EVERY current setting — all tabs plus the current '
                'engine — as the boot default (default.yaml)? The app will '
                'load these on every launch.'):
            return
        def go():
            ok = self.client.call_save_default()
            if ok:
                self._set_status('SAVED AS DEFAULT', '#3366aa')
                self._refresh_profile_list()
            else:
                messagebox.showerror('Save failed', 'save_as_default service failed.')
        threading.Thread(target=go, daemon=True).start()

    def _on_load_speed_preset(self, slug):
        """v5.1 (A6a + presets): load a built-in speed preset (slow/medium/fast)
        through the node's hardened load_profile service on a worker thread -
        the SAME path as custom presets. load_profile applies each param one at
        a time with type/range tolerance, so a partial YAML can't silently drop
        a key the way the old live _apply_preset (per-key set_value) did. Runs
        off the Tk thread so a slow service call can't freeze the UI."""
        self._set_status(f'LOADING {slug} preset...', '#aa6633')
        def go():
            ok = self.client.call_load_profile(f'{slug}.yaml')
            if ok:
                self._set_status(f'{slug.upper()} PRESET LOADED', '#3366aa')
            else:
                messagebox.showerror(
                    'Preset load failed',
                    f'Could not load the {slug} preset. Is {slug}.yaml present '
                    f'in the profiles dir? (install.sh seeds it next to '
                    f'default.yaml.)')
                self._set_status('PRESET LOAD FAILED', '#cc3333')
        threading.Thread(target=go, daemon=True).start()

    # ---- Camera viewer process management -----------------------------

    def _cam_topic(self):
        t = self.cam_topic_entry.get().strip() or '/custom_sortingv5/image_result'
        if not t.startswith('/'):
            t = '/' + t
        return t

    def _set_cam_status(self, text, color):
        self.cam_status_var.set(text)
        self.cam_status_label.configure(bg=color)

    def _close_viewer_proc(self):
        """Terminate the currently tracked viewer subprocess if alive.
        Best-effort - we send SIGTERM then SIGKILL if it doesn't die."""
        import signal
        proc = self._viewer_proc
        self._viewer_proc = None
        if proc is None:
            return
        if proc.poll() is not None:
            return  # already dead
        try:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                try: proc.wait(timeout=1.0)
                except Exception: pass
        except Exception:
            pass

    def _spawn_viewer(self, cmd, label):
        def go():
            self._set_cam_status('opening...', '#3366aa')
            self._close_viewer_proc()
            try:
                self._viewer_proc = subprocess.Popen(
                    cmd, stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True)
                self._set_cam_status(f'{label} (pid={self._viewer_proc.pid})',
                                     '#2e8b57')
                # Surface to the launcher terminal too - users were missing
                # spawn results because the tuner status bar is easy to miss.
                print(f'[viewer] spawned {label} (pid={self._viewer_proc.pid}) '
                      f'cmd={cmd[0]} ...', flush=True)
            except FileNotFoundError:
                self._set_cam_status(f'{cmd[0]} not installed', '#aa3333')
                print(f'[viewer] spawn FAILED: {cmd[0]} not on PATH', flush=True)
                messagebox.showerror(
                    'Viewer not found',
                    f"Could not start '{cmd[0]}'.\n"
                    f"Install it or use a different viewer.")
            except Exception as e:
                self._set_cam_status('spawn failed', '#aa3333')
                print(f'[viewer] spawn FAILED: {type(e).__name__}: {e}', flush=True)
                messagebox.showerror('Viewer error', f'{type(e).__name__}: {e}')
        threading.Thread(target=go, daemon=True).start()

    @staticmethod
    def _viewer_env_prefix():
        # Round 2 set QT_X11_NO_MITSHM=1 + QT_QPA_PLATFORM=xcb +
        # LIBGL_ALWAYS_SOFTWARE=1 to fight a blank-window issue. Those
        # three together force Qt off the GPU and onto CPU software
        # rendering, which throttles 640x480 BGR8 paint to ~5-10 fps.
        # The viewer-blank issue was actually rqt_image_view not being
        # invoked via `ros2 run`. That's fixed since round 5, so the
        # safe-mode env vars are no longer needed.
        #
        # Set JETARM_V5_QT_SAFE=1 in ~/.jetarm_v5.env to restore
        # the safe-mode fallback if Qt mis-renders on a different host.
        if os.environ.get('JETARM_V5_QT_SAFE', '0') == '1':
            return ("export QT_X11_NO_MITSHM=1; "
                    "export QT_QPA_PLATFORM=xcb; "
                    "export LIBGL_ALWAYS_SOFTWARE=1; ")
        return ""

    def _on_open_rqt(self):
        # rqt_image_view is a ROS 2 ament Python plugin - not a system
        # binary on PATH. Invoke via `ros2 run` so it actually launches.
        # image_transport:=raw disables the default compressed/theora/
        # compressedDepth plugin negotiation - if we leave the default,
        # the orbbec depth_cam publisher tries to JPEG-encode 16UC1
        # depth (and theora-encode it, and compressedDepth-encode RGB)
        # which floods the log at ~30 Hz and eventually corrupts the
        # orbbec onNewFrameSetCallback buffer (OpenCV "Failed to
        # allocate 227 TB" -> container crash).
        # tee output to /tmp so the user can `cat` it if rqt errors.
        bash_cmd = (
            self._viewer_env_prefix()
            + "source /opt/ros/humble/setup.bash; "
            + "source ~/ros2_ws/install/setup.bash; "
            + f"ros2 run rqt_image_view rqt_image_view {self._cam_topic()} "
            + "--ros-args -p image_transport:=raw "
            + "2>&1 | tee /tmp/jetarm_v5_rqt.log; exec bash"
        )
        self._spawn_viewer(['terminator', '-x', 'bash', '-c', bash_cmd], 'rqt_image_view')

    def _on_open_image_view(self):
        # Same reason as _on_open_rqt: image_transport:=raw to skip the
        # broken compressed/theora plugin pipeline.
        bash_cmd = (
            self._viewer_env_prefix()
            + "source /opt/ros/humble/setup.bash; "
            + "source ~/ros2_ws/install/setup.bash; "
            + f"ros2 run image_view image_view --ros-args -r image:={self._cam_topic()} "
            + "-p image_transport:=raw "
            + "2>&1 | tee /tmp/jetarm_v5_image_view.log; exec bash"
        )
        self._spawn_viewer(['terminator', '-x', 'bash', '-c', bash_cmd], 'image_view')

    def _on_open_browser(self):
        # Browser is a separate path: we don't track a subprocess to "close"
        # (the user closes the tab). Still kill any GUI viewer that was open
        # so we genuinely "swap".
        def go():
            self._close_viewer_proc()
            self._set_cam_status('opening browser...', '#3366aa')
            try:
                ip = self._detect_ip()
                # type=ros_compressed streams the node's pre-encoded JPEG
                # sibling (<topic>/compressed): no server-side re-encode in
                # web_video_server and ~10x less node->server bandwidth than
                # the raw Image topic. Quality knob: publish_jpeg_quality.
                url = (f'http://{ip}:8080/stream_viewer?topic={self._cam_topic()}'
                       f'&type=ros_compressed')
                opener = self._pick_browser()
                if opener is None:
                    messagebox.showinfo(
                        'No browser found',
                        f"Open this URL manually:\n\n{url}")
                    self._set_cam_status(f'open manually: {url}', '#aa6633')
                    return
                subprocess.Popen([opener, url],
                                 stdin=subprocess.DEVNULL,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL,
                                 start_new_session=True)
                self._set_cam_status(f'browser @ {ip}:8080', '#2e8b57')
            except Exception as e:
                self._set_cam_status('browser failed', '#aa3333')
                messagebox.showerror('Browser error', f'{type(e).__name__}: {e}')
        threading.Thread(target=go, daemon=True).start()

    def _on_close_viewer(self):
        def go():
            self._close_viewer_proc()
            self._set_cam_status('closed', '#666666')
        threading.Thread(target=go, daemon=True).start()

    @staticmethod
    def _detect_ip():
        try:
            out = subprocess.check_output(['hostname', '-I'], timeout=2).decode()
            ip = out.strip().split()[0] if out.strip() else ''
            if ip and not ip.startswith('127.'):
                return ip
        except Exception:
            pass
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.connect(('1.1.1.1', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return 'localhost'

    @staticmethod
    def _pick_browser():
        import shutil
        for b in ('xdg-open', 'sensible-browser', 'firefox',
                  'chromium', 'chromium-browser', 'google-chrome'):
            if shutil.which(b):
                return b
        return None

    def run(self):
        self.root.mainloop()


def main():
    # rclpy / ros2 launch always pass `--ros-args ...` to every Node
    # executable. argparse rejects unknown args by default, which crashes
    # the UI process on startup. Use parse_known_args and discard the rest.
    ap = argparse.ArgumentParser()
    ap.add_argument('--node-name', default='custom_sortingv5')
    # Round 16 LL.4: pop-out calibration window. When set to a tab name
    # (position/color/depth) the UI builds ONLY the calibration notebook
    # and raises that tab. Used by the "Open calibration in separate
    # window" button, which spawns a fresh process on the same domain.
    ap.add_argument('--calib-window', default='', choices=['', 'position',
                                                           'color', 'depth'])
    args, _ros_args = ap.parse_known_args()
    rclpy.init()
    # Unique client node name for pop-out windows so a second UI process
    # doesn't clash with the main tuner's rclpy node name.
    client = TunerClient(args.node_name,
                         client_name=('tuner_calib_%d' % os.getpid()
                                      if args.calib_window else None))
    if not client.wait_ready(timeout=10.0):
        print(f'No parameter services for /{args.node_name}', file=sys.stderr)
        rclpy.shutdown(); sys.exit(1)
    threading.Thread(target=rclpy.spin, args=(client,), daemon=True).start()
    ui = TunerUI(client, calib_only=args.calib_window or None)
    threading.Thread(target=client.call_enable_sorting, args=(False,),
                     daemon=True).start()
    try:
        ui.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
