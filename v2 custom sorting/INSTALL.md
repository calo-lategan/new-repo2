# Custom Sorting v2 - install on the Jetson Orin Nano

This folder contains:

| File | Purpose |
|------|---------|
| `custom_sortingv2.py` | Faster, self-calibrating sorting node with vision + bus-servo grip feedback. |
| `custom_sorting_nodev2.launch.py` | Launch file - brings up SDK, depth camera, and the v2 node. |
| `tune_ui.py` | Tkinter UI to live-tune every behavior knob via the ROS2 parameter API. |

## What changed vs v1

- **Speed**: trajectory interpolation everywhere, base joint dispatched in parallel with the arm, fixed `time.sleep` pads replaced with scaled durations, lock-in thresholds dropped (5/8 vs 10/10).
- **Accuracy**: startup self-calibration nudges each colored bin's `place_position` based on what the camera actually sees; multi-frame averaging before locking a pick; pre-grasp hover-and-recheck.
- **Dynamic gripping**: after every close we read `GetBusServoState` for servo 10. If the position is at "fully closed" the grab missed -> retry tighter / lower. If the close stalls short (load) we back off. After lift the camera re-checks the spot - if the object is still there, retry.
- **Tunable**: every knob is a ROS2 parameter with `on_set_parameters_callback`, so `tune_ui.py` (or `rqt_reconfigure`) can change speed, aggression, retries, gripper pulses, area gates, etc. live.

## Install on the robot

1. Copy the two main files into the `app` ROS2 package on the Jetson:

   ```bash
   cp custom_sortingv2.py            ~/ros2_ws/src/app/app/custom_sortingv2.py
   cp custom_sorting_nodev2.launch.py ~/ros2_ws/src/app/launch/custom_sorting_nodev2.launch.py
   cp tune_ui.py                     ~/ros2_ws/src/app/app/tune_ui.py
   ```

2. Add the entry points to `~/ros2_ws/src/app/setup.py` inside `entry_points -> console_scripts`:

   ```python
   'custom_sortingv2 = app.custom_sortingv2:main',
   'tune_ui          = app.tune_ui:main',
   ```

3. Build and source:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select app
   source install/setup.bash
   ```

4. Launch:

   ```bash
   ros2 launch app custom_sorting_nodev2.launch.py
   ```

   Useful overrides:

   ```bash
   ros2 launch app custom_sorting_nodev2.launch.py \
        engine_path:=/home/ubuntu/third_party_ros2/data/best_scaff2.engine \
        motion_speed:=1.6 aggression:=1.4 \
        servo_feedback_enabled:=true vision_confirm_pick:=true \
        startup_self_calibrate:=true
   ```

5. Open the live tuner in another terminal:

   ```bash
   ros2 run app tune_ui
   # or:  python3 ~/ros2_ws/src/app/app/tune_ui.py
   ```

## Notes / gotchas

- The node defaults to the YOLO engine at `/home/ubuntu/third_party_ros2/data/best_scaff2.engine`. Override with the `engine_path` launch argument or set the parameter at runtime.
- If the `ros_robot_controller/bus_servo/get_state` service isn't up within 5s the node logs a warning and falls back to **vision-only** retries (no servo feedback).
- `~/recalibrate` is a `Trigger` service - call it at any time to re-run the bin self-calibration:
  ```bash
  ros2 service call /custom_sortingv2/recalibrate std_srvs/srv/Trigger
  ```
- `tune_ui.py` shows three preset buttons (Slow & safe / Default / Fast & aggressive) plus per-knob sliders. Changes are pushed to the node immediately.
- On servo overheating (>65 C reported by `GetBusServoState`) the gripper logic treats the close as "stalled" and backs off, which protects the gripper servo during long sorting runs.
