import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    # Default to paused so the user has to press START in the tuner UI before
    # any motion happens - keeps the robot safe while you calibrate / tune.
    start = LaunchConfiguration('start', default='false')
    start_arg = DeclareLaunchArgument(
        'start', default_value=start,
        description='If true, sorting begins immediately on launch (no UI gate).')
    auto_tune_ui = LaunchConfiguration('tune_ui', default='true')
    auto_tune_ui_arg = DeclareLaunchArgument(
        'tune_ui', default_value=auto_tune_ui,
        description='Spawn the live parameter tuner UI alongside the node.')
    display = LaunchConfiguration('display', default='true')
    display_arg = DeclareLaunchArgument('display', default_value=display)
    broadcast = LaunchConfiguration('broadcast', default='false')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
    engine_path = LaunchConfiguration(
        'engine_path',
        default='/home/ubuntu/third_party_ros2/data/best_scaff2.engine')
    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value=engine_path,
        description='Path to the YOLO TensorRT engine used for scaff detection.')
    motion_speed = LaunchConfiguration('motion_speed', default='1.4')
    motion_speed_arg = DeclareLaunchArgument(
        'motion_speed', default_value=motion_speed,
        description='Speed multiplier (1.0 = v1 baseline, >1 = faster).')
    aggression = LaunchConfiguration('aggression', default='1.2')
    aggression_arg = DeclareLaunchArgument(
        'aggression', default_value=aggression,
        description='Trajectory aggression - higher = larger interpolation steps.')
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

    custom_sorting_v2_node = Node(
        package='app',
        executable='custom_sortingv2',
        output='screen',
        parameters=[{
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

    # Tuner UI - co-launches with the node so the operator can calibrate /
    # tune / Start+Stop without a second terminal. Disable with tune_ui:=false
    # for headless / SSH runs without an X display.
    tune_ui_node = Node(
        package='app',
        executable='tune_ui',
        output='screen',
        condition=IfCondition(auto_tune_ui),
    )

    return [
        start_arg,
        auto_tune_ui_arg,
        display_arg,
        broadcast_arg,
        engine_path_arg,
        motion_speed_arg,
        aggression_arg,
        servo_feedback_arg,
        vision_confirm_arg,
        self_calibrate_arg,
        sdk_launch,
        depth_camera_launch,
        custom_sorting_v2_node,
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
