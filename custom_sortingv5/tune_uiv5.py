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
import subprocess
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path

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
    'grasp_short_axis_min_ratio': ('Short-axis gate', '|w-h|/max(w,h) above which short-axis preference applies (cubes: stay vendor)'),
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
# Calibrate tab: manual world-XY offset + workspace scale.
# grip_offset_x/y were on Detection in commit Q; commit R moves them here
# (their proper home) alongside workspace_scale.
CALIB_FLOAT_PARAMS = [
    ('grip_offset_x',        -0.10, 0.10, 0.005),
    ('grip_offset_y',        -0.10, 0.10, 0.005),
    ('workspace_scale',       0.50, 1.50, 0.01),
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
    'inference_warmup',
    'hot_log_inference_ms',
    # Independent stop/start of camera subscription + inference.
    'enable_camera_sub',
    'enable_inference',
]

# Built-in quick presets (code constants - applied live, never saved over).
# Custom namable presets are saved as full profiles in PROFILES_DIR; their
# names must not collide with a built-in slug or 'default'/'yolo'.
BUILTIN_PRESETS = [
    ('Slow & safe', {'motion_speed': 0.7, 'aggression': 0.7,
                     'gripper_settle': 0.8, 'gripper_close_duration': 0.6}),
    ('Default', {'motion_speed': 1.5, 'aggression': 1.3,
                 'gripper_settle': 0.5, 'gripper_close_duration': 0.35}),
    ('Fast & aggressive', {'motion_speed': 2.1, 'aggression': 1.7,
                           'gripper_settle': 0.3, 'gripper_close_duration': 0.25}),
    ('Precision', {'motion_speed': 0.9, 'aggression': 0.8,
                   'count_still_threshold': 8, 'detection_avg_frames': 6,
                   'gripper_settle': 0.8}),
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


# Names a custom preset may not take (would clobber a built-in or the boot file).
RESERVED_PRESET_SLUGS = (
    {_slugify_preset(lbl) for lbl, _ in BUILTIN_PRESETS} | {'default', 'yolo'}
)


class TunerClient(Node):
    def __init__(self, target_node):
        super().__init__('custom_sortingv5_tuner')
        self.target = target_node
        self.set_cli = self.create_client(SetParameters, f'/{target_node}/set_parameters')
        self.get_cli = self.create_client(GetParameters, f'/{target_node}/get_parameters')
        self.list_cli = self.create_client(ListParameters, f'/{target_node}/list_parameters')
        self.enable_cli = self.create_client(SetBool, f'/{target_node}/enable_sorting')
        self.recalibrate_cli = self.create_client(Trigger, f'/{target_node}/recalibrate')
        self.run_calibration_cli = self.create_client(Trigger, f'/{target_node}/run_calibration')
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
        # Live heartbeat mirror: the node publishes its 5s heartbeat as
        # JSON on ~/status. The background rclpy.spin thread services this
        # subscription; the UI polls latest_status via root.after().
        self.latest_status = None
        self.status_rx_t = 0.0
        self.status_sub = self.create_subscription(
            String, f'/{target_node}/status', self._on_status, 1)

    def _on_status(self, msg):
        # Runs on the background spin thread: only write plain attributes
        # here (thread-safe); all Tk updates happen in the UI's after() poll.
        try:
            self.latest_status = json.loads(msg.data)
            self.status_rx_t = time.time()
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
        self._wait_future(future, 2.0)

    def _trigger(self, client):
        if not client.wait_for_service(timeout_sec=1.0): return False
        future = client.call_async(Trigger.Request())
        self._wait_future(future, 2.0)
        return future.done() and future.result() is not None and future.result().success

    def _set_string_bool(self, client, s, b=True):
        if not client.wait_for_service(timeout_sec=1.0): return False
        req = SetStringBool.Request(); req.data_str = s; req.data_bool = b
        future = client.call_async(req)
        self._wait_future(future, 4.0)
        return future.done() and future.result() is not None and future.result().success

    def call_enable_sorting(self, enable):
        if not self.enable_cli.wait_for_service(timeout_sec=1.0): return False
        req = SetBool.Request(); req.data = bool(enable)
        future = self.enable_cli.call_async(req)
        self._wait_future(future, 2.0)
        return future.done() and future.result() is not None

    def call_recalibrate(self): return self._trigger(self.recalibrate_cli)
    def call_run_calibration(self): return self._trigger(self.run_calibration_cli)
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


class TunerUI:
    def __init__(self, client):
        self.client = client
        self.root = tk.Tk()
        self.root.title('JetArm v5 - live tuner')
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
            age = time.time() - self.client.status_rx_t
            if st is None:
                pass  # node hasn't published yet - keep the placeholder
            elif age > 12.0:
                self.perf_var.set(f'perf: NO HEARTBEAT for {age:.0f}s - node down?')
            else:
                unmapped = int(st.get('unmapped_count', 0) or 0)
                badge = f"  unmapped={unmapped}" if unmapped else ''
                self.perf_var.set(
                    f"perf: cam={st.get('cam_fps', '-')}fps "
                    f"pub={st.get('pub_fps', '-')}fps "
                    f"ai={st.get('ai', '?')} "
                    f"inf_age={st.get('inference_age_ms', '-')}ms{badge}")
                engine = st.get('engine') or ''
                task = st.get('task') or ''
                override = (st.get('task_override') or 'auto').strip().lower()
                if engine:
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
                if cur != 'CALIBRATING...':
                    if node_running and not cur.startswith('RUNNING'):
                        self._set_status('RUNNING (node)', '#2e8b57')
                    elif not node_running and cur.startswith('RUNNING'):
                        self._set_status('STOPPED (node)', '#aa3333')
        except Exception:
            pass  # never let the poll loop die
        self.root.after(1000, self._poll_node_status)

    def _build(self):
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
        calibrate_tab = ttk.Frame(notebook); notebook.add(calibrate_tab, text='Calibrate')
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
        # Calibrate tab gets its own Save & Close bar (also clears the live
        # overlay mode on save). Body is added below to the scrollable area.
        calib_keys = [n for n, *_ in CALIB_FLOAT_PARAMS]
        self._add_calibrate_save_bar(calibrate_tab, calib_keys)

        # Scrollable content bodies so nothing is clipped on a small Jetson
        # screen (this is the fix for "I can't scroll" / "can't reach the
        # engine picker"). Content goes into the *body*, not the tab frame.
        speed_body = self._make_scrollable(speed_tab)
        grip_body = self._make_scrollable(grip_tab)
        detect_body = self._make_scrollable(detect_tab)
        toggles_body = self._make_scrollable(toggles_tab)
        places_body = self._make_scrollable(places_tab)
        calibrate_body = self._make_scrollable(calibrate_tab)

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
        # Calibrate tab — manual world XY offset + workspace scale + overlay.
        self._build_calibrate_tab(calibrate_body, current)
        # Profiles tab is short and has its own listbox scroll - no body wrap.
        self._build_profiles_tab(profiles_tab)

        # Presets bar: built-in quick presets + namable custom presets.
        preset = ttk.LabelFrame(self.root, text='Presets')
        preset.pack(fill='x', padx=8, pady=4)
        for label, mapping in BUILTIN_PRESETS:
            ttk.Button(preset, text=label,
                       command=lambda m=mapping: self._apply_preset(m)
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

    def _build_calibrate_tab(self, parent, current):
        """Three world sliders + Enter-manual / Reset buttons. Save & Close
        lives in the bottom bar."""
        header = ttk.LabelFrame(parent, text='Manual workspace calibration')
        header.pack(fill='x', padx=8, pady=(8, 4))
        ttk.Label(header, foreground='#666', wraplength=720,
                  justify='left',
                  text='Nudge how the arm interprets the workspace, without '
                       'the AprilTag. X / Y shift the landing point in '
                       'metres; Z scales the workspace map around the '
                       'centre. Press "Enter manual mode" to show the live '
                       'workspace overlay on the camera view; "Save & Close" '
                       'persists the values to default.yaml and clears the '
                       'overlay. Use the top-bar CALIBRATE button to run the '
                       'AprilTag auto-calibration instead.'
                  ).pack(anchor='w', padx=8, pady=(2, 6))

        knob_frame = ttk.LabelFrame(parent, text='World offsets (live)')
        knob_frame.pack(fill='x', padx=8, pady=4)
        for name, lo, hi, res in CALIB_FLOAT_PARAMS:
            self._add_float(knob_frame, name, lo, hi, res,
                            float(current.get(name, lo)))

        btn_frame = ttk.LabelFrame(parent, text='Overlay')
        btn_frame.pack(fill='x', padx=8, pady=4)
        ttk.Label(btn_frame, foreground='#666',
                  text='The overlay shows the workspace centre, X/Y axes, '
                       'and how the offset moves the arm\'s belief.'
                  ).pack(anchor='w', padx=8, pady=(2, 4))
        row = ttk.Frame(btn_frame); row.pack(fill='x', padx=4, pady=4)
        tk.Button(row, text='Enter manual mode', bg='#3366aa', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_enter_manual_calibrate
                  ).pack(side='left', padx=4)
        tk.Button(row, text='Reset to 0', bg='#aa6633', fg='white',
                  font=('TkDefaultFont', 10, 'bold'),
                  command=self._on_calibrate_reset
                  ).pack(side='left', padx=4)

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
        """Zero the three calibrate sliders (live; not yet persisted)."""
        setters = getattr(self, '_param_setters', {})
        for name, default in (('grip_offset_x', 0.0),
                              ('grip_offset_y', 0.0),
                              ('workspace_scale', 1.0)):
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
        try:
            persisted = json.loads(
                self.client.get_values(['yolo_enabled_classes'])
                .get('yolo_enabled_classes', '[]') or '[]')
            persisted_set = set(int(x) for x in persisted) if persisted else None
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
        """User preset files in PROFILES_DIR, excluding the boot default.yaml."""
        if not PROFILES_DIR.exists():
            return []
        return [p.name for p in sorted(PROFILES_DIR.glob('*.yaml'))
                if p.name != 'default.yaml']

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
        self.status_var.set(text)
        self.status_label.configure(bg=color)

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
        # Vendor AprilTag calibration: STOPS sorting and MOVES the arm.
        if not messagebox.askyesno(
                'Run calibration?',
                'This STOPS sorting and MOVES the arm to the calibration '
                'pose, then runs AprilTag calibration.\n\n'
                'Requires AprilTags (IDs 1/2/3, fallback 100; 2.5 cm) flat '
                'and fully in the camera view.\n\nContinue?'):
            return
        def go():
            self.client.call_enable_sorting(False)
            self._set_status('CALIBRATING...', '#3366aa')
            ok = self.client.call_run_calibration()
            self._set_status('STOPPED (calibrated)' if ok else 'CALIBRATE FAILED',
                             '#aa3333' if ok else '#aa6633')
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

    def _apply_preset(self, mapping):
        # Update the on-screen widget first so the UI reflects the preset
        # (previously presets pushed to ROS but the sliders kept showing
        # their old values), then push to the node.
        for k, v in mapping.items():
            setter = self._param_setters.get(k)
            if setter is not None:
                setter(v)
            self.client.set_value(k, v)

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
    args, _ros_args = ap.parse_known_args()
    rclpy.init()
    client = TunerClient(args.node_name)
    if not client.wait_ready(timeout=10.0):
        print(f'No parameter services for /{args.node_name}', file=sys.stderr)
        rclpy.shutdown(); sys.exit(1)
    threading.Thread(target=rclpy.spin, args=(client,), daemon=True).start()
    ui = TunerUI(client)
    threading.Thread(target=client.call_enable_sorting, args=(False,),
                     daemon=True).start()
    try:
        ui.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
