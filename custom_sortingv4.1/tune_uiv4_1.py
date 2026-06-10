#!/usr/bin/env python3
# coding: utf8
# Live tuner UI for custom_sortingv4_1.
#
# Adds over v2's tune_ui:
#  - Engine hot-swap: file picker / dropdown of available .engine files.
#    Calls /custom_sortingv4_1/load_engine - the inference worker swaps
#    between frames, no restart needed.
#  - Profile manager: list, save, load named profiles from
#    ~/jetarm_v4_profiles/.
#  - "Save as default" button - persists the current settings as the
#    profile loaded on every startup.
#  - All v2 controls (Start / Stop / Calibrate, sliders, presets).
#  - Tabs reorganised: Control / Speed / Grip / Vision / Models / Profiles / Toggles.

import os
import sys
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
from interfaces.srv import SetStringBool


PROFILES_DIR = Path(os.environ.get('JETARM_V4_PROFILES',
                                   str(Path.home() / 'jetarm_v4_profiles')))
DEFAULT_ENGINES_DIR = Path(os.environ.get('JETARM_V4_ENGINES_DIR',
                                          '/home/ubuntu/third_party_ros2/data'))


FLOAT_PARAMS = [
    ('motion_speed',          0.3, 2.5, 0.05),
    ('aggression',            0.3, 2.0, 0.05),
    ('hover_height',          0.02, 0.15, 0.005),
    ('approach_dwell',        0.0, 1.0, 0.05),
    ('lock_distance_thresh',  0.001, 0.05, 0.001),
    ('gripper_close_duration', 0.1, 2.0, 0.05),
    # Round 15b: jaw-clamp settle before lift (pick) / retreat (place), and
    # how far below the detected z the descend goes. Both mirror the
    # reference pick_and_place timings/geometry.
    ('gripper_settle',        0.0, 1.5, 0.05),
    ('grab_depth',            0.0, 0.05, 0.001),
    # Round 7: YOLO knobs + throttles.
    ('yolo_conf_thresh',      0.05, 0.95, 0.01),
    ('yolo_iou_thresh',       0.10, 0.90, 0.01),
    ('inference_max_hz',      0.0,  60.0, 1.0),   # 0 = uncapped
    ('publish_max_hz',        0.0,  60.0, 1.0),   # 0 = uncapped
    ('publish_scale',         0.25, 1.0, 0.05),   # downsample before publish
]

INT_PARAMS = [
    ('min_object_area',          50, 5000, 50),
    ('max_object_area',          1000, 30000, 250),
    ('count_still_threshold',    1, 30, 1),
    ('count_move_threshold',     1, 30, 1),
    ('detection_avg_frames',     1, 10, 1),
    ('gripper_open_pulse',       50, 500, 5),
    ('gripper_close_pulse',      300, 700, 5),
    # Round 15: gripper_full_closed_pulse, gripper_slack, gripper_step_pulse,
    # max_pick_retries are no longer used — pick is one-shot. Params remain
    # declared on the node for backward profile compat but are hidden here.
    # Round 7: YOLO knobs.
    # NB: yolo_imgsz is intentionally NOT here. TensorRT engines have
    # a fixed input shape baked in at compile time (best_scaff3.engine
    # is 320x320). To change imgsz you must re-export from Ultralytics
    # at the new size and rebuild the .engine.
    ('yolo_max_det',             1, 300, 1),
]

BOOL_PARAMS = [
    'parallel_base_motion',
    # Round 15: vision_confirm_pick + servo_feedback_enabled hidden — pick
    # is one-shot, both are dead. Declared on the node for backward profile
    # compat.
    'startup_self_calibrate',
    'place_bin_color_check',
    'inference_warmup',
    'hot_log_inference_ms',
    # Round 7: independent stop/start of camera subscription + inference.
    'enable_camera_sub',
    'enable_inference',
]


class TunerClient(Node):
    def __init__(self, target_node):
        super().__init__('custom_sortingv4_1_tuner')
        self.target = target_node
        self.set_cli = self.create_client(SetParameters, f'/{target_node}/set_parameters')
        self.get_cli = self.create_client(GetParameters, f'/{target_node}/get_parameters')
        self.list_cli = self.create_client(ListParameters, f'/{target_node}/list_parameters')
        self.enable_cli = self.create_client(SetBool, f'/{target_node}/enable_sorting')
        self.recalibrate_cli = self.create_client(Trigger, f'/{target_node}/recalibrate')
        self.enter_cli = self.create_client(Trigger, f'/{target_node}/enter')
        self.exit_cli = self.create_client(Trigger, f'/{target_node}/exit')
        self.load_engine_cli = self.create_client(SetStringBool, f'/{target_node}/load_engine')
        self.save_profile_cli = self.create_client(SetStringBool, f'/{target_node}/save_profile')
        self.load_profile_cli = self.create_client(SetStringBool, f'/{target_node}/load_profile')
        self.save_default_cli = self.create_client(Trigger, f'/{target_node}/save_as_default')

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
    def call_enter(self):       return self._trigger(self.enter_cli)
    def call_exit(self):        return self._trigger(self.exit_cli)
    def call_save_default(self): return self._trigger(self.save_default_cli)

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
        self.root.title('JetArm v4.1 - live tuner')
        self.root.geometry('760x1100')
        # name -> callable(value) that updates that param's widget locally
        # (no ROS traffic). Used by presets to keep the UI in sync.
        self._param_setters = {}
        self._building = True
        self._build()
        self._building = False

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
        self.savedef_btn = tk.Button(btn_row, text='SAVE AS DEFAULT', bg='#666688',
                                     fg='white', font=('TkDefaultFont', 10, 'bold'),
                                     width=16, height=2, command=self._on_save_default)
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
        self.cam_topic_entry.insert(0, '/custom_sortingv4_1/image_result')
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

        speed_tab = ttk.Frame(notebook); notebook.add(speed_tab, text='Speed / motion')
        grip_tab = ttk.Frame(notebook); notebook.add(grip_tab, text='Grip')
        vision_tab = ttk.Frame(notebook); notebook.add(vision_tab, text='Vision')
        toggles_tab = ttk.Frame(notebook); notebook.add(toggles_tab, text='Toggles')
        models_tab = ttk.Frame(notebook); notebook.add(models_tab, text='Models')
        profiles_tab = ttk.Frame(notebook); notebook.add(profiles_tab, text='Profiles')

        all_names = ([n for n, *_ in FLOAT_PARAMS] + [n for n, *_ in INT_PARAMS]
                     + BOOL_PARAMS + ['engine_path'])
        current = self.client.get_values(all_names)
        self.engine_var.set(self._short_engine(current.get('engine_path', '-')))

        speed_names = {'motion_speed', 'aggression', 'hover_height',
                       'approach_dwell', 'gripper_close_duration'}
        # Only the grip params that still exist post-round-15 (one-shot pick:
        # full_closed/slack/step/retries are gone from the UI).
        grip_names = {'gripper_open_pulse', 'gripper_close_pulse',
                      'gripper_settle', 'grab_depth'}
        vision_names = {'min_object_area', 'max_object_area',
                        'count_still_threshold', 'count_move_threshold',
                        'detection_avg_frames', 'lock_distance_thresh'}

        # Packed first so it renders at the top of the Vision tab.
        ttk.Label(vision_tab, foreground='#774400', wraplength=700,
                  justify='left',
                  text='YOLO knobs (yolo_conf_thresh / yolo_iou_thresh / '
                       'yolo_max_det / inference_max_hz) apply INSTANTLY '
                       'while STOPPED - watch the boxes change in the live '
                       'viewer. While RUNNING they are queued and land when '
                       'you press STOP, so a slider can\'t change detection '
                       'mid-pick.'
                  ).pack(anchor='w', padx=8, pady=(8, 6))

        for name, lo, hi, res in FLOAT_PARAMS:
            parent = (speed_tab if name in speed_names else
                      grip_tab if name in grip_names else vision_tab)
            self._add_float(parent, name, lo, hi, res, current.get(name, lo))
        for name, lo, hi, res in INT_PARAMS:
            parent = grip_tab if name in grip_names else vision_tab
            self._add_int(parent, name, lo, hi, res, int(current.get(name, lo)))
        for name in BOOL_PARAMS:
            self._add_bool(toggles_tab, name, bool(current.get(name, False)))

        self._build_models_tab(models_tab, current.get('engine_path', ''))
        self._build_profiles_tab(profiles_tab)

        # Presets bar
        preset = ttk.LabelFrame(self.root, text='Presets')
        preset.pack(fill='x', padx=8, pady=4)
        ttk.Button(preset, text='Slow & safe',
                   command=lambda: self._apply_preset({'motion_speed': 0.7,
                                                        'aggression': 0.7,
                                                        'gripper_settle': 0.8,
                                                        'gripper_close_duration': 0.6})
                   ).pack(side='left', padx=4, pady=4)
        ttk.Button(preset, text='Default',
                   command=lambda: self._apply_preset({'motion_speed': 1.5,
                                                        'aggression': 1.3,
                                                        'gripper_settle': 0.5,
                                                        'gripper_close_duration': 0.35})
                   ).pack(side='left', padx=4, pady=4)
        ttk.Button(preset, text='Fast & aggressive',
                   command=lambda: self._apply_preset({'motion_speed': 2.1,
                                                        'aggression': 1.7,
                                                        'gripper_settle': 0.3,
                                                        'gripper_close_duration': 0.25})
                   ).pack(side='left', padx=4, pady=4)
        ttk.Button(preset, text='Precision',
                   command=lambda: self._apply_preset({'motion_speed': 0.9,
                                                        'aggression': 0.8,
                                                        'count_still_threshold': 8,
                                                        'detection_avg_frames': 6,
                                                        'gripper_settle': 0.8})
                   ).pack(side='left', padx=4, pady=4)

    def _short_engine(self, path):
        if not path or path == '-': return '-'
        return os.path.basename(path)

    # ---- generic widgets ----

    def _add_float(self, parent, name, lo, hi, res, init):
        self._add_numeric(parent, name, lo, hi, res, init, kind='float')

    def _add_int(self, parent, name, lo, hi, res, init):
        self._add_numeric(parent, name, lo, hi, res, init, kind='int')

    def _add_numeric(self, parent, name, lo, hi, res, init, kind):
        """One slider row with:
          - label (name)
          - slider that updates the local display on drag, but only
            pushes to ROS on mouse-release (sliderReleased() pattern).
            Stops the executor saturation we saw in the v4.1 logs
            (hundreds of /set_parameters calls per drag).
          - numeric entry box for exact values (type then Enter or tab).
            Both paths produce exactly one ROS call per finalized value.
        """
        frame = ttk.Frame(parent); frame.pack(fill='x', padx=6, pady=3)
        ttk.Label(frame, text=name, width=28).pack(side='left')

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

    def _add_bool(self, parent, name, init):
        var = tk.BooleanVar(value=bool(init))
        def on_change():
            if not self._building:
                self.client.set_value(name, bool(var.get()))
        ttk.Checkbutton(parent, text=name, variable=var, command=on_change
                        ).pack(anchor='w', padx=12, pady=4)
        self._param_setters[name] = lambda v: var.set(bool(v))

    # ---- Models tab ----

    def _build_models_tab(self, parent, current_path):
        # Track the live engine path so Refresh marks the CURRENT engine
        # [active] - a lambda closing over current_path froze the marker at
        # the boot engine after hot-swaps.
        self._active_engine = current_path
        ttk.Label(parent,
                  text=f'Discovered .engine files in {DEFAULT_ENGINES_DIR}:',
                  foreground='#444').pack(anchor='w', padx=8, pady=(8, 4))
        listbox_frame = ttk.Frame(parent); listbox_frame.pack(fill='both', expand=True,
                                                              padx=8, pady=4)
        self.engine_listbox = tk.Listbox(listbox_frame, height=8)
        self.engine_listbox.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(listbox_frame, orient='vertical',
                                  command=self.engine_listbox.yview)
        scrollbar.pack(side='right', fill='y')
        self.engine_listbox.configure(yscrollcommand=scrollbar.set)
        self._refresh_engine_list(current_path)

        btn_row = ttk.Frame(parent); btn_row.pack(fill='x', padx=8, pady=6)
        ttk.Button(btn_row, text='Refresh',
                   command=lambda: self._refresh_engine_list(self._active_engine)
                   ).pack(side='left', padx=4)
        ttk.Button(btn_row, text='Load selected',
                   command=self._on_load_selected_engine).pack(side='left', padx=4)
        ttk.Button(btn_row, text='Browse for .engine...',
                   command=self._on_browse_engine).pack(side='left', padx=4)

        ttk.Label(parent, text='Manual path:',
                  foreground='#444').pack(anchor='w', padx=8, pady=(8, 2))
        self.engine_entry = ttk.Entry(parent)
        self.engine_entry.pack(fill='x', padx=8)
        self.engine_entry.insert(0, current_path or '')
        ttk.Button(parent, text='Load entered path',
                   command=self._on_load_entry).pack(anchor='e', padx=8, pady=4)

    def _refresh_engine_list(self, current_path):
        self.engine_listbox.delete(0, tk.END)
        if not DEFAULT_ENGINES_DIR.exists():
            self.engine_listbox.insert(tk.END, f'(dir not found: {DEFAULT_ENGINES_DIR})')
            return
        engines = sorted(DEFAULT_ENGINES_DIR.glob('*.engine'))
        if not engines:
            self.engine_listbox.insert(tk.END, '(no .engine files)')
            return
        for p in engines:
            marker = ' [active]' if str(p) == current_path else ''
            self.engine_listbox.insert(tk.END, f'{p.name}{marker}')

    def _on_load_selected_engine(self):
        sel = self.engine_listbox.curselection()
        if not sel: return
        text = self.engine_listbox.get(sel[0]).split(' [active]')[0]
        path = str(DEFAULT_ENGINES_DIR / text)
        self._do_engine_swap(path)

    def _on_browse_engine(self):
        path = filedialog.askopenfilename(
            title='Pick .engine file', initialdir=str(DEFAULT_ENGINES_DIR),
            filetypes=[('TensorRT engine', '*.engine'), ('All files', '*.*')])
        if path:
            self._do_engine_swap(path)

    def _on_load_entry(self):
        path = self.engine_entry.get().strip()
        if path: self._do_engine_swap(path)

    def _do_engine_swap(self, path):
        def go():
            ok = self.client.call_load_engine(path)
            if ok:
                self._active_engine = path
                self.engine_var.set(self._short_engine(path))
                self._refresh_engine_list(path)
                self._set_status('ENGINE SWAPPED', '#3366aa')
            else:
                messagebox.showerror('Engine swap failed',
                                     f'Could not load:\n{path}')
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
        def go():
            ok = self.client.call_save_profile(name)
            if ok:
                self._refresh_profile_list()
                self._set_status('PROFILE SAVED', '#3366aa')
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
        def go():
            self.client.call_enable_sorting(False)
            self._set_status('CALIBRATING...', '#3366aa')
            ok = self.client.call_recalibrate()
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
        t = self.cam_topic_entry.get().strip() or '/custom_sortingv4_1/image_result'
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
        # Set JETARM_V4_1_QT_SAFE=1 in ~/.jetarm_v4_1.env to restore
        # the safe-mode fallback if Qt mis-renders on a different host.
        if os.environ.get('JETARM_V4_1_QT_SAFE', '0') == '1':
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
            + "2>&1 | tee /tmp/jetarm_v4_1_rqt.log; exec bash"
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
            + "2>&1 | tee /tmp/jetarm_v4_1_image_view.log; exec bash"
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
                url = f'http://{ip}:8080/stream_viewer?topic={self._cam_topic()}'
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
    ap.add_argument('--node-name', default='custom_sortingv4_1')
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
