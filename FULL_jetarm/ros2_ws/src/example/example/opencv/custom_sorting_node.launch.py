import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    start = LaunchConfiguration('start', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    
    # CHANGED TO FALSE: This stops OpenCV cv2.imshow from crashing the container
    display = LaunchConfiguration('display', default='false') 
    display_arg = DeclareLaunchArgument('display', default_value=display)
    
    broadcast = LaunchConfiguration('broadcast', default='false')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)

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

    # CALLING YOUR CUSTOM SCRIPT HERE!
    custom_sorting_node = Node(
        package='app',
        executable='custom_sorting', 
        output='screen',
        parameters=[{ 'start': start, 'display': display, 'broadcast': broadcast}]
    )

    # ADDED: ROS2 Native Camera Viewer to bypass OpenCV crashes
    # This automatically opens the window and tunes into your YOLO feed
    camera_viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/custom_sorting/image_result'],
        output='screen'
    )

    return [start_arg,
            display_arg,
            broadcast_arg,
            sdk_launch,
            depth_camera_launch,
            custom_sorting_node,
            camera_viewer_node,  # <--- Added to the launch list!
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
