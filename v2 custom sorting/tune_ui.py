#!/usr/bin/env python3
# coding: utf8
# Live parameter tuner for custom_sortingv2.
#
# Tkinter UI with sliders/checkboxes for every behavior knob in
# custom_sortingv2.py. Talks to the running node via the ROS2 parameter
# service (rcl_interfaces.srv.SetParameters) so changes apply instantly,
# no restart needed.
#
# Run on the Jetson (with a display attached) after launching the v2 node:
#   ros2 launch app custom_sorting_nodev2.launch.py
#   ros2 run app tune_ui                # if registered via setup.py
#   # or:  python3 tune_ui.py
#
# It connects to the node `custom_sortingv2` by default - override with
# --node-name if you renamed it.

import sys
import argparse
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger, SetBool


FLOAT_PARAMS = [
    # (name, min, max, resolution)
    ('motion_speed',          0.3, 2.5, 0.05),
    ('aggression',            0.3, 2.0, 0.05),
    ('hover_height',          0.02, 0.15, 0.005),
    ('approach_dwell',        0.0, 1.0, 0.05),
    ('lock_distance_thresh',  0.001, 0.05, 0.001),
    ('gripper_close_duration', 0.1, 2.0, 0.05),
]

INT_PARAMS = [
    ('min_object_area',          50, 5000, 50),
    ('max_object_area',          1000, 30000, 250),
    ('count_still_threshold',    1, 30, 1),
    ('count_move_threshold',     1, 30, 1),
    ('detection_avg_frames',     1, 10, 1),
    ('gripper_open_pulse',       50, 500, 5),
    ('gripper_close_pulse',      300, 700, 5),
    ('gripper_full_closed_pulse', 500, 900, 5),
    ('gripper_slack',            5, 80, 1),
    ('gripper_step_pulse',       5, 100, 1),
    ('max_pick_retries',         0, 6, 1),
]

BOOL_PARAMS = [
    'parallel_base_motion',
    'vision_confirm_pick',
    'servo_feedback_enabled',
    'startup_self_calibrate',
    'place_bin_color_check',
]


class TunerClient(Node):
    def __init__(self, target_node):
        super().__init__('custom_sortingv2_tuner')
        self.target = target_node
        self.set_cli = self.create_client(SetParameters,
                                          f'/{target_node}/set_parameters')
        self.get_cli = self.create_client(GetParameters,
                                          f'/{target_node}/get_parameters')
        self.list_cli = self.create_client(ListParameters,
                                           f'/{target_node}/list_parameters')
        # Lifecycle / control services exposed by custom_sortingv2.
        self.enable_cli = self.create_client(SetBool,
                                             f'/{target_node}/enable_sorting')
        self.recalibrate_cli = self.create_client(Trigger,
                                                  f'/{target_node}/recalibrate')
        self.enter_cli = self.create_client(Trigger, f'/{target_node}/enter')
        self.exit_cli = self.create_client(Trigger, f'/{target_node}/exit')

    def wait_ready(self, timeout=10.0):
        return (self.set_cli.wait_for_service(timeout_sec=timeout)
                and self.get_cli.wait_for_service(timeout_sec=timeout))

    def get_values(self, names):
        req = GetParameters.Request(); req.names = list(names)
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
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

    def call_enable_sorting(self, enable):
        if not self.enable_cli.wait_for_service(timeout_sec=1.0):
            return False
        req = SetBool.Request(); req.data = bool(enable)
        future = self.enable_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None

    def call_recalibrate(self):
        if not self.recalibrate_cli.wait_for_service(timeout_sec=1.0):
            return False
        future = self.recalibrate_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None

    def call_enter(self):
        if not self.enter_cli.wait_for_service(timeout_sec=1.0):
            return False
        future = self.enter_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None

    def call_exit(self):
        if not self.exit_cli.wait_for_service(timeout_sec=1.0):
            return False
        future = self.exit_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None

    def set_value(self, name, value):
        p = Parameter(); p.name = name
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        else:
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)
        p.value = pv
        req = SetParameters.Request(); req.parameters = [p]
        future = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


class TunerUI:
    def __init__(self, client):
        self.client = client
        self.root = tk.Tk()
        self.root.title('JetArm v2 - live tuner')
        self.root.geometry('620x920')
        self._building = True
        self._build()
        self._building = False

    def _build(self):
        # ---- Top control bar: Start / Stop / Calibrate ----
        ctrl = ttk.LabelFrame(self.root, text='Robot control')
        ctrl.pack(fill='x', padx=8, pady=(8, 4))

        # Status indicator with color
        self.status_var = tk.StringVar(value='STOPPED')
        status_row = ttk.Frame(ctrl); status_row.pack(fill='x', padx=6, pady=4)
        ttk.Label(status_row, text='Status:').pack(side='left')
        self.status_label = tk.Label(status_row, textvariable=self.status_var,
                                     fg='white', bg='#aa3333',
                                     font=('TkDefaultFont', 11, 'bold'),
                                     padx=10, pady=2)
        self.status_label.pack(side='left', padx=8)

        btn_row = ttk.Frame(ctrl); btn_row.pack(fill='x', padx=6, pady=4)

        # Use big tk.Button (not ttk) so colors actually apply on most themes.
        self.start_btn = tk.Button(btn_row, text='START SORTING',
                                   bg='#2e8b57', fg='white',
                                   font=('TkDefaultFont', 11, 'bold'),
                                   width=16, height=2, command=self._on_start)
        self.start_btn.pack(side='left', padx=4)

        self.stop_btn = tk.Button(btn_row, text='STOP',
                                  bg='#aa3333', fg='white',
                                  font=('TkDefaultFont', 11, 'bold'),
                                  width=12, height=2, command=self._on_stop)
        self.stop_btn.pack(side='left', padx=4)

        self.cal_btn = tk.Button(btn_row, text='CALIBRATE',
                                 bg='#3366aa', fg='white',
                                 font=('TkDefaultFont', 11, 'bold'),
                                 width=12, height=2, command=self._on_calibrate)
        self.cal_btn.pack(side='left', padx=4)

        # Hint label
        hint = ttk.Label(ctrl, foreground='#555555',
                         text='Tip: STOP halts vision + motion. Adjust sliders or '
                              'CALIBRATE while stopped, then START again.')
        hint.pack(anchor='w', padx=8, pady=(0, 4))

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=8, pady=8)

        speed_tab = ttk.Frame(notebook); notebook.add(speed_tab, text='Speed / motion')
        grip_tab = ttk.Frame(notebook); notebook.add(grip_tab, text='Grip / retries')
        vision_tab = ttk.Frame(notebook); notebook.add(vision_tab, text='Vision')
        toggles_tab = ttk.Frame(notebook); notebook.add(toggles_tab, text='Toggles')

        # Pull current values once so sliders start in the right place.
        all_names = ([n for n, *_ in FLOAT_PARAMS] + [n for n, *_ in INT_PARAMS]
                     + BOOL_PARAMS)
        current = self.client.get_values(all_names)

        speed_names = {'motion_speed', 'aggression', 'hover_height', 'approach_dwell',
                       'gripper_close_duration'}
        grip_names = {'gripper_open_pulse', 'gripper_close_pulse',
                      'gripper_full_closed_pulse', 'gripper_slack',
                      'gripper_step_pulse', 'max_pick_retries',
                      'gripper_close_duration'}
        vision_names = {'min_object_area', 'max_object_area',
                        'count_still_threshold', 'count_move_threshold',
                        'detection_avg_frames', 'lock_distance_thresh'}

        def add_float(parent, name, lo, hi, res):
            self._add_float(parent, name, lo, hi, res, current.get(name, lo))

        def add_int(parent, name, lo, hi, res):
            self._add_int(parent, name, lo, hi, res, int(current.get(name, lo)))

        for name, lo, hi, res in FLOAT_PARAMS:
            parent = speed_tab if name in speed_names else \
                     grip_tab if name in grip_names else vision_tab
            add_float(parent, name, lo, hi, res)
        for name, lo, hi, res in INT_PARAMS:
            parent = grip_tab if name in grip_names else vision_tab
            add_int(parent, name, lo, hi, res)
        for name in BOOL_PARAMS:
            self._add_bool(toggles_tab, name, bool(current.get(name, False)))

        # Quick presets
        preset = ttk.LabelFrame(self.root, text='Presets')
        preset.pack(fill='x', padx=8, pady=4)
        ttk.Button(preset, text='Slow & safe',
                   command=lambda: self._apply_preset({'motion_speed': 0.7,
                                                        'aggression': 0.7,
                                                        'max_pick_retries': 4,
                                                        'gripper_close_duration': 0.6})
                   ).pack(side='left', padx=4, pady=4)
        ttk.Button(preset, text='Default',
                   command=lambda: self._apply_preset({'motion_speed': 1.4,
                                                        'aggression': 1.2,
                                                        'max_pick_retries': 3,
                                                        'gripper_close_duration': 0.4})
                   ).pack(side='left', padx=4, pady=4)
        ttk.Button(preset, text='Fast & aggressive',
                   command=lambda: self._apply_preset({'motion_speed': 2.0,
                                                        'aggression': 1.7,
                                                        'max_pick_retries': 2,
                                                        'gripper_close_duration': 0.25})
                   ).pack(side='left', padx=4, pady=4)

    def _add_float(self, parent, name, lo, hi, res, init):
        frame = ttk.Frame(parent); frame.pack(fill='x', padx=6, pady=3)
        ttk.Label(frame, text=name, width=28).pack(side='left')
        val_label = ttk.Label(frame, text=f'{init:.3f}', width=8)
        val_label.pack(side='right')
        var = tk.DoubleVar(value=float(init))
        def on_change(_=None):
            v = float(var.get())
            val_label.configure(text=f'{v:.3f}')
            if not self._building:
                self.client.set_value(name, v)
        scale = ttk.Scale(frame, from_=lo, to=hi, variable=var,
                          orient='horizontal', command=on_change)
        scale.pack(side='left', fill='x', expand=True, padx=8)

    def _add_int(self, parent, name, lo, hi, res, init):
        frame = ttk.Frame(parent); frame.pack(fill='x', padx=6, pady=3)
        ttk.Label(frame, text=name, width=28).pack(side='left')
        var = tk.IntVar(value=int(init))
        val_label = ttk.Label(frame, text=str(init), width=8)
        val_label.pack(side='right')
        def on_change(_=None):
            v = int(round(float(var.get())))
            val_label.configure(text=str(v))
            if not self._building:
                self.client.set_value(name, v)
        scale = ttk.Scale(frame, from_=lo, to=hi, variable=var,
                          orient='horizontal', command=on_change)
        scale.pack(side='left', fill='x', expand=True, padx=8)

    def _add_bool(self, parent, name, init):
        var = tk.BooleanVar(value=bool(init))
        def on_change():
            if not self._building:
                self.client.set_value(name, bool(var.get()))
        cb = ttk.Checkbutton(parent, text=name, variable=var, command=on_change)
        cb.pack(anchor='w', padx=12, pady=4)

    def _apply_preset(self, mapping):
        for k, v in mapping.items():
            self.client.set_value(k, v)

    # ------------------------------------------------------------------ control buttons

    def _set_status(self, text, color):
        self.status_var.set(text)
        self.status_label.configure(bg=color)

    def _on_start(self):
        # Run service calls in a worker thread so the UI doesn't freeze if
        # the node is briefly unresponsive.
        def go():
            ok_enter = self.client.call_enter()  # idempotent on the v2 node
            ok = self.client.call_enable_sorting(True)
            if ok and ok_enter:
                self._set_status('RUNNING', '#2e8b57')
            else:
                self._set_status('NODE NOT REACHABLE', '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    def _on_stop(self):
        def go():
            ok = self.client.call_enable_sorting(False)
            if ok:
                self._set_status('STOPPED', '#aa3333')
            else:
                self._set_status('NODE NOT REACHABLE', '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    def _on_calibrate(self):
        def go():
            # Force a stop first - calibrating during motion is unsafe.
            self.client.call_enable_sorting(False)
            self._set_status('CALIBRATING...', '#3366aa')
            ok = self.client.call_recalibrate()
            if ok:
                self._set_status('STOPPED (calibrated)', '#aa3333')
            else:
                self._set_status('CALIBRATE FAILED', '#aa6633')
        threading.Thread(target=go, daemon=True).start()

    def run(self):
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--node-name', default='custom_sortingv2',
                    help='Name of the running sorting node (default: custom_sortingv2)')
    args = ap.parse_args()

    rclpy.init()
    client = TunerClient(args.node_name)
    if not client.wait_ready(timeout=10.0):
        print(f'No parameter services for /{args.node_name} - is the node running?',
              file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    # Spin client in a background thread so service calls work.
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    ui = TunerUI(client)

    # Safety: assert "stopped" state on UI startup so the robot doesn't begin
    # sorting just because we connected. The user has to explicitly press
    # START before any motion happens.
    threading.Thread(target=client.call_enable_sorting, args=(False,),
                     daemon=True).start()

    try:
        ui.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
