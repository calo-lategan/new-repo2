import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
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
        executable='custom_sortingv4',
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
        }]
    )

    tune_ui_node = Node(
        package='app',
        executable='tune_uiv4',
        output='screen',
        condition=IfCondition(auto_tune_ui),
    )

    return [
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
        sdk_launch,
        depth_camera_launch,
        custom_sorting_v4_node,
        tune_ui_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
