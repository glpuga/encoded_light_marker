from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            remappings=[
                ('/image_raw', '/sensor_input/image_raw'),
                ('/camera_info', '/sensor_input/camera_info'),
            ]
        ),
        Node(
            package='elm_mock',
            executable='elm_mock_node',
            remappings=[
                ('/elm_mock_node/input/image_raw', '/sensor_input/image_raw'),
                ('/elm_mock_node/input/camera_info', '/sensor_input/camera_info'),
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
