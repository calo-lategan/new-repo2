import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction,
    ExecuteProcess, TimerAction,
)
from launch.conditions import IfCondition


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    # Boots paused so the user has to press START in the tuner UI.
    start = LaunchConfiguration('start', default='false')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    auto_tune_ui = LaunchConfiguration('tune_ui', default='true')
    auto_tune_ui_arg = DeclareLaunchArgument(
        'tune_ui', default_value=auto_tune_ui,
        description='Spawn the v4 live tuner UI alongside the node.')
    display = LaunchConfiguration('display', default='true')
    display_arg = DeclareLaunchArgument('display', default_value=display)
    broadcast = LaunchConfiguration('broadcast', default='false')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
    engine_path = LaunchConfiguration(
        'engine_path',
        default='/home/ubuntu/third_party_ros2/data/best_scaff2.engine')
    engine_path_arg = DeclareLaunchArgument(
        'engine_path', default_value=engine_path,
        description='Initial YOLO TensorRT engine. Hot-swap at runtime via UI.')

    # Optional named profile to load on launch (looks in ~/jetarm_v4_profiles).
    # The default profile (default.yaml) is auto-loaded by the node itself if
    # present, so this arg is mainly for "boot into 'fast'", "boot into 'slow'".
    profile = LaunchConfiguration('profile', default='')
    profile_arg = DeclareLaunchArgument(
        'profile', default_value=profile,
        description='Profile name to load on startup (empty = default.yaml only).')

    # Per-tunable shortcuts so you can override at the CLI without writing YAML.
    motion_speed = LaunchConfiguration('motion_speed', default='1.5')
    motion_speed_arg = DeclareLaunchArgument('motion_speed', default_value=motion_speed)
    aggression = LaunchConfiguration('aggression', default='1.3')
    aggression_arg = DeclareLaunchArgument('aggression', default_value=aggression)
    servo_feedback = LaunchConfiguration('servo_feedback_enabled', default='true')
    servo_feedback_arg = DeclareLaunchArgument(
        'servo_feedback_enabled', default_value=servo_feedback)
    vision_confirm = LaunchConfiguration('vision_confirm_pick', default='true')
    vision_confirm_arg = DeclareLaunchArgument(
        'vision_confirm_pick', default_value=vision_confirm)
    self_calibrate = LaunchConfiguration('startup_self_calibrate', default='true')
    self_calibrate_arg = DeclareLaunchArgument(
        'startup_self_calibrate', default_value=self_calibrate)
    debug = LaunchConfiguration('debug', default='false')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value=debug,
        description='Verbose stage logging from custom_sortingv4_1 + tune_uiv4_1.')

    # Auto-open a desktop window showing the annotated camera feed. Default
    # is on. Falls back gracefully if neither rqt_image_view nor image_view
    # are available.
    # Auto-open the image_view_chain (rqt -> image_view -> browser) at
    # launch. The tuner UI buttons swap/replace the viewer after launch.
    # Pass `image_view:=false` if you don't want the auto-popup.
    open_image_view = LaunchConfiguration('image_view', default='true')
    open_image_view_arg = DeclareLaunchArgument(
        'image_view', default_value=open_image_view,
        description='Auto-open the image_view_chain on launch '
                    '(default true; pass image_view:=false to disable).')
    image_view_topic = LaunchConfiguration(
        'image_view_topic', default='/custom_sortingv4_1/image_result')
    image_view_topic_arg = DeclareLaunchArgument(
        'image_view_topic', default_value=image_view_topic,
        description='Topic to show in the auto-opened image viewer.')

    # We INCLUDE sdk_launch and depth_camera_launch so our launch is
    # self-sufficient: even if Hiwonder's start_app_node.service hasn't
    # successfully brought everything up yet (or its sub-launches died),
    # our launch will bring its own copies of the SDK + depth camera.
    # bringup also launches these but both share the same package paths,
    # and the ament-index resolution for orbbec_camera works AS LONG AS
    # the service is running (which the launcher ensures via
    # `systemctl start`). When bringup also tries to launch them, one
    # side wins and the other quietly fails - which is what the working
    # earlier versions relied on. Don't remove these includes again.
    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )
    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')),
    )

    # --- Resolve the optional named profile to a YAML file path -----------
    # If the user passed `profile:=fast`, look for ~/jetarm_v4_profiles/fast.yaml
    # and pass it via parameters=[...]. The node also auto-loads default.yaml
    # at startup independently, so the precedence ends up:
    #   hardcoded defaults < default.yaml < profile.yaml < CLI overrides.
    extra_params = []
    profile_value = profile.perform(context).strip()
    if profile_value:
        candidate = os.path.expanduser(
            f'~/jetarm_v4_profiles/{profile_value}.yaml'
            if not profile_value.endswith('.yaml')
            else f'~/jetarm_v4_profiles/{profile_value}')
        if os.path.isfile(candidate):
            extra_params.append(candidate)
        else:
            print(f'[launch] WARNING: profile file {candidate} not found, ignoring')

    custom_sorting_v4_node = Node(
        package='app',
        executable='custom_sortingv4_1',
        output='screen',
        parameters=extra_params + [{
            'start': start,
            'display': display,
            'broadcast': broadcast,
            'engine_path': engine_path,
            'motion_speed': motion_speed,
            'aggression': aggression,
            'servo_feedback_enabled': servo_feedback,
            'vision_confirm_pick': vision_confirm,
            'startup_self_calibrate': self_calibrate,
            'debug': debug,
        }]
    )

    tune_ui_node = Node(
        package='app',
        executable='tune_uiv4_1',
        output='screen',
        condition=IfCondition(auto_tune_ui),
    )

    # web_video_server: serves http://<host>:8080/stream?topic=... for the
    # browser viewer. Was previously brought up by Hiwonder's factory
    # start_app_node.service, which we now permanently disable on launch
    # - so we have to start it ourselves.
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='log',
        parameters=[{
            'port': 8080,
            'default_stream_type': 'mjpeg',
            'verbose': False,
        }],
    )

    # --- Auto-opened desktop image viewer (with browser fallback) ---------
    # Delegate to the image_view_chain.sh wrapper which:
    #   1. Tries rqt_image_view <topic>
    #   2. Falls back to `ros2 run image_view image_view`
    #   3. When the GUI viewer exits (closed by user OR failed to spawn),
    #      auto-opens the system browser at
    #         http://<jetson-ip>:8080/stream?topic=<topic>
    #      using hostname -I to figure out the IP.
    # That way the user never needs to look up an IP, AND if they close
    # the GUI window they still have a working view in the browser.
    image_view_topic_value = image_view_topic.perform(context).strip()
    chain_script = os.path.expanduser('~/jetarm_v4_1/image_view_chain.sh')
    chain_present = os.path.exists(chain_script)
    if not chain_present:
        print(f'[launch] WARNING: {chain_script} not present - '
              f'image viewer auto-open disabled. Re-run install.sh or '
              f'see INSTALL.md.')

    image_viewer_action = None
    if chain_present:
        # Delay 6s so the node has booted, the YOLO engine has warmed up,
        # and the ~/image_result topic is being published. Otherwise the
        # viewer opens a window that just says "no image".
        # 8s (was 6s) so web_video_server has time to bind :8080 before
        # the chain script's probe_web_server hits the URL.
        image_viewer_action = TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[chain_script, image_view_topic_value],
                    output='screen',
                    condition=IfCondition(open_image_view),
                )
            ],
        )

    actions = [
        start_arg,
        auto_tune_ui_arg,
        display_arg,
        broadcast_arg,
        engine_path_arg,
        profile_arg,
        motion_speed_arg,
        aggression_arg,
        servo_feedback_arg,
        vision_confirm_arg,
        self_calibrate_arg,
        debug_arg,
        open_image_view_arg,
        image_view_topic_arg,
        sdk_launch,
        depth_camera_launch,
        custom_sorting_v4_node,
        tune_ui_node,
        web_video_server_node,
    ]
    if image_viewer_action is not None:
        actions.append(image_viewer_action)
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
