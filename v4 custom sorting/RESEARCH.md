# v4 design research notes

This is the condensed research that drove the v4 design. Each section maps to a concrete change in `custom_sortingv4.py` or `tune_uiv4.py`.

## 1. Executor + callback group patterns (rclpy)

- `SingleThreadedExecutor` (default) runs one callback at a time. Any `time.sleep` or sync service call stalls *everything* — including the camera sub.
- `MultiThreadedExecutor(num_threads=N)` dispatches across a thread pool. Required as soon as you have one long-running callback (inference) plus timers/services that must keep responding.
- `MutuallyExclusiveCallbackGroup` (default per-node): only one callback in the group runs at a time. Safe-by-default but serializes work.
- `ReentrantCallbackGroup`: callbacks may run in parallel. Use for the *client side* of a service call you make from another callback.
- **Classic deadlock**: calling `client.call(req)` (sync) or `spin_until_future_complete(...)` from inside a callback while the timer/sub and the client share a `MutuallyExclusiveCallbackGroup`. The spinner is stuck in your callback, so the response future is never serviced. Fix: client in `ReentrantCallbackGroup` + `MultiThreadedExecutor`, or use `call_async` and return.
- Never call `rclpy.spin*` from inside a callback. Never block the executor thread waiting on ROS work.

**v4 applies this**: `MultiThreadedExecutor(num_threads=4)`. Camera sub on `MutuallyExclusiveCallbackGroup` (one frame at a time, no race on shared state). All service clients (kinematics, bus_servo_state, profile/engine swap) on `ReentrantCallbackGroup`.

## 2. Zero-copy / intra-process comms

- True zero-copy intra-process is C++ (`rclcpp`) only. `rclpy` doesn't get pointer-passing; ros2/design issue #251 is still open.
- `ComposableNode` + `use_intra_process_comms=True` reduces serialization between *C++* components in one process.
- `image_transport` republishes Image over plugins (`compressed`, `compressedDepth`, `theora`). Over localhost on the Orin, raw is usually faster than JPEG decode.
- **cv_bridge alternative**: `np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)` is a *view* — no copy. `cv_bridge.imgmsg_to_cv2` copies.

**v4 applies this**: `image_callback` uses `np.frombuffer().reshape()` instead of cv_bridge. Saves a memcpy per frame. Output Image is built directly from the numpy buffer with `bgr.tobytes()` — also no cv_bridge.

## 3. Image pipeline efficiency

- `rclpy.qos.qos_profile_sensor_data` for camera subs: BEST_EFFORT, KEEP_LAST 5. Matches what most camera drivers publish; default RELIABLE causes stalls.
- Bounded queues — depth=1 means we always work the freshest frame.
- BEST_EFFORT skips retransmits — correct for high-rate streams.

**v4 applies this**: image and camera_info subscriptions use `qos_profile_sensor_data`. The InferenceWorker holds at most one frame and overwrites; never queues lag.

## 4. Async service calls without blocking spin

- `client.call_async(req)` returns a `rclpy.task.Future`; attach `future.add_done_callback(cb)` and *return*.
- `spin_until_future_complete` deadlocks inside a callback because the executor thread is already in your callback and cannot service the response. Only safe from `main()` outside spin.

**v4 applies this**: `MotionController.goto_pose` uses `kinematics_client.call_async(...)` then a small spin-free polling helper that *only runs from the dedicated transport thread* (never from an executor callback). The transport thread isn't an executor thread, so no deadlock.

## 5. YOLO / TensorRT inference loop

- One CUDA context per process. Build `YOLO('model.engine')` once on the thread that will run inference.
- Ultralytics' `BasePredictor` holds a `threading.Lock` (`self._lock`) — concurrent `model.predict` calls serialize anyway, so a thread pool buys nothing.
- Warmup: run one dummy `model(np.zeros((H,W,3), np.uint8))` after load — first inference includes engine deserialization and is 10–100× slower.
- Latency vs throughput: pick loop wants batch=1 with the smallest input size that still detects.
- Hot-swap `.engine`: `self.model = YOLO(new_path, task='detect')` is fine *only* from the inference thread (or while holding a swap lock the inference thread checks between frames). Swapping mid-`predict` will race on CUDA state.
- Drop the old model reference and `torch.cuda.empty_cache()` before loading a new engine — important on 8 GB Orin.

**v4 applies this**: dedicated `InferenceWorker(threading.Thread)` owns the model. ROS service `~/load_engine` writes a pending path; the worker swaps it between frames, runs a warmup, calls `torch.cuda.empty_cache()` after dropping the old reference.

## 6. Parameter persistence / hot-reload

- Declare every param via `declare_parameter('name', default)`; read via `get_parameter('name').value`.
- ROS2 YAML format:
  ```yaml
  /**:
    ros__parameters:
      key: value
  ```
  `/**` matches any node name/namespace.
- Load at launch: `Node(..., parameters=[yaml_file])` or CLI `--ros-args --params-file file.yaml`.
- Runtime save: `ros2 param dump /node_name > out.yaml`. Programmatic dump: iterate via `list_parameters` + `get_parameters` and write YAML yourself.
- `add_on_set_parameters_callback(cb)` fires on every `ros2 param set`.

**v4 applies this**: every tunable is a declared param with a range descriptor. Profiles live in `~/jetarm_v4_profiles/` as `/**: ros__parameters:` YAML. Three services: `~/save_profile <name>`, `~/load_profile <name>`, `~/save_as_default`. Default profile auto-loaded on startup. The launch file accepts `profile:=fast` and resolves it to the matching YAML.

## 7. What to avoid (silent slowdowns)

- `time.sleep` in a callback — use a `create_timer` instead.
- Creating a publisher / subscriber / `CvBridge()` per callback — do it in `__init__`.
- `cv_bridge.imgmsg_to_cv2` when `np.frombuffer(...).reshape(...)` would be a free view.
- Default RELIABLE/KEEP_LAST 10 QoS on a camera topic — stalls under bursts.
- Logging at INFO inside the hot loop — `get_logger().info(...)` is surprisingly expensive (formatting + DDS `/rosout` publish).

**v4 applies this**: no per-frame INFO logs (only WARN on failures). No per-callback object creation. cv_bridge removed. QoS `qos_profile_sensor_data`.

---

## Net deltas vs v2 (concrete)

| Concern | v2 | v4 |
|---|---|---|
| Executor | Multi-threaded but no explicit groups | Multi-threaded (4 thr), explicit groups |
| Image conversion | `cv_bridge.imgmsg_to_cv2` (copy) | `np.frombuffer(...).reshape(...)` (view) |
| Camera QoS | default (RELIABLE/depth 10) | `qos_profile_sensor_data` (BEST_EFFORT/depth 5) |
| Inference | sync inside the camera-driven loop | dedicated worker thread, frame slot, latest-only |
| Engine swap | restart only | `~/load_engine` service, swaps between frames + warmup + `torch.cuda.empty_cache()` |
| Settings | live-tunable but lost on restart | live-tunable + named profiles + `default.yaml` auto-loaded |
| Per-target tuning | uniform | `target_overrides` JSON for per-label speed/grip |
| Per-frame INFO log | yes | no (toggleable inference-ms HUD overlay instead) |
| Retry recovery | yes (vision + servo) | yes (same logic, faster cadence) |

## Sources

- [Deadlocks in rclpy and how to prevent them with use of callback groups — Karelics](https://karelics.fi/deadlocks-in-rclpy/)
- [Using Callback Groups — ROS 2 docs](https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html)
- [Avoiding Race Conditions and Deadlocks — Hello Robot Stretch docs](https://docs.hello-robot.com/0.3/ros2/avoiding_deadlocks_race_conditions/)
- [Quality of Service settings — ROS 2 Humble](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 QoS design article](https://design.ros2.org/articles/qos.html)
- [image_transport package](https://index.ros.org/p/image_transport/)
- [Intra-Process Communications for all language clients — ros2/design #251](https://github.com/ros2/design/issues/251)
- [Composing multiple nodes in a single process — ROS 2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html)
- [Cut ROS 2 Latency by 60% with Component Nodes](https://markaicode.com/ros2-component-nodes-zero-copy-transport/)
- [Thread-Safe Inference with YOLO Models — Ultralytics](https://docs.ultralytics.com/guides/yolo-thread-safe-inference/)
- [Ultralytics BasePredictor reference](https://docs.ultralytics.com/reference/engine/predictor/)
- [Ultralytics TensorRT backend reference](https://docs.ultralytics.com/reference/nn/backends/tensorrt/)
- [Understanding parameters — ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [ROS2 YAML for parameters — Robotics Back-End](https://roboticsbackend.com/ros2-yaml-params/)
- [How to Use ROS 2 Parameters — Foxglove](https://foxglove.dev/blog/how-to-use-ros2-parameters)
