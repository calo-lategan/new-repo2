# Custom Sorting v2 - install on the Jetson Orin Nano

This folder contains:

| File | Purpose |
|------|---------|
| `custom_sortingv2.py` | Faster, self-calibrating sorting node with vision + bus-servo grip feedback. |
| `custom_sorting_nodev2.launch.py` | Launch file - brings up SDK, depth camera, the v2 node, and the tuner UI. |
| `tune_ui.py` | Tkinter UI with **START / STOP / CALIBRATE** buttons and live sliders for every behavior knob. |
| `launch_v2.sh` | One-click shell launcher (sources ROS, sets env, runs the launch file). |
| `jetarm-sort-v2.desktop` | Desktop shortcut that runs `launch_v2.sh`. |
| `setup.py.snippet` | Lines to paste into the `app` package's `setup.py`. |

## What changed vs v1

- **Speed**: trajectory interpolation everywhere, base joint dispatched in parallel with the arm, fixed `time.sleep` pads replaced with scaled durations, lock-in thresholds dropped (5/8 vs 10/10).
- **Accuracy**: startup self-calibration nudges each colored bin's `place_position` based on what the camera actually sees; multi-frame averaging before locking a pick; pre-grasp hover-and-recheck.
- **Dynamic gripping**: after every close we read `GetBusServoState` for servo 10. If the position is at "fully closed" the grab missed -> retry tighter / lower. If the close stalls short (load) we back off. After lift the camera re-checks the spot - if the object is still there, retry.
- **Tunable + safe-by-default**: the launch file now boots the node **paused** (`start:=false`) and auto-spawns the tuner UI. Nothing moves until you press **START** in the UI. Press **STOP** any time to halt vision + motion so you can adjust sliders or recalibrate without breaking anything.

## Install on the robot

1. Copy the files into the `app` ROS2 package on the Jetson:

   ```bash
   cp custom_sortingv2.py            ~/ros2_ws/src/app/app/custom_sortingv2.py
   cp tune_ui.py                     ~/ros2_ws/src/app/app/tune_ui.py
   cp custom_sorting_nodev2.launch.py ~/ros2_ws/src/app/launch/custom_sorting_nodev2.launch.py
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

4. Launch (node + tuner UI open together):

   ```bash
   ros2 launch app custom_sorting_nodev2.launch.py
   ```

   Useful overrides:

   ```bash
   ros2 launch app custom_sorting_nodev2.launch.py \
        engine_path:=/home/ubuntu/third_party_ros2/data/best_scaff2.engine \
        motion_speed:=1.6 aggression:=1.4 \
        tune_ui:=true start:=false \
        servo_feedback_enabled:=true vision_confirm_pick:=true \
        startup_self_calibrate:=true
   ```

   - `tune_ui:=false` - run headless / over SSH without an X display.
   - `start:=true` - skip the START button and begin sorting immediately.

## One-click desktop launcher

So you can launch the whole stack from the desktop:

```bash
mkdir -p ~/jetarm_v2
cp launch_v2.sh ~/jetarm_v2/launch_v2.sh
chmod +x ~/jetarm_v2/launch_v2.sh

# Drop the shortcut on the desktop AND in the apps menu.
cp jetarm-sort-v2.desktop ~/Desktop/jetarm-sort-v2.desktop
chmod +x ~/Desktop/jetarm-sort-v2.desktop
mkdir -p ~/.local/share/applications
cp jetarm-sort-v2.desktop ~/.local/share/applications/
```

On Ubuntu 24.04 (GNOME), right-click the icon on the desktop -> *Allow Launching* the first time. Double-click after that to start the whole stack: SDK, depth camera, sorting node, and the tuner UI all open in one terminal window.

The shell script exports `CAMERA_TYPE=GEMINI`, `CHASSIS_TYPE=Slide_Rails`, `need_compile=False` by default; override them in your shell before launching if your hardware differs.

## Operating it (the safe loop)

1. Click the desktop shortcut (or run `ros2 launch app custom_sorting_nodev2.launch.py`). The node boots **STOPPED** and the tuner UI opens.
2. (Optional) Press **CALIBRATE** to re-run the bin self-calibration with the current scene.
3. Adjust sliders (motion_speed, aggression, gripper_close_pulse, etc.) or pick a preset (Slow & safe / Default / Fast & aggressive).
4. Press **START SORTING** when ready.
5. Press **STOP** any time to pause; the robot freezes, vision goes quiet. Tweak settings or recalibrate freely. Press **START** again to resume.

## Notes / gotchas

- The node defaults to the YOLO engine at `/home/ubuntu/third_party_ros2/data/best_scaff2.engine`. Override with the `engine_path` launch argument or set the parameter at runtime.
- If the `ros_robot_controller/bus_servo/get_state` service isn't up within 5s the node logs a warning and falls back to **vision-only** retries (no servo feedback).
- `~/recalibrate`, `~/enable_sorting`, `~/enter`, `~/exit` are all callable directly with `ros2 service call ...` if you want to script the robot from outside the UI.
- On servo overheating (>65 C reported by `GetBusServoState`) the gripper logic treats the close as "stalled" and backs off, which protects the gripper servo during long sorting runs.
