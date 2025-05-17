from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera nodes and Processesing nodes come in pairs, 
        # i.e. each camera connects to one processing node

        # Camera 0
        Node(
            package='camera_package',
            executable='camera_node',
            name='camera0_node',
            parameters=[{'camera_id': 0}],
            remappings=[('image_raw', '/camera0/image_raw')]
        ),
        Node(
            package='camera_package',
            executable='processing_node',
            name='processor0_node',
            remappings=[
                ('image_raw', '/camera0/image_raw'),
                ('image_processed', '/camera0/image_processed'),
                ('aruco_detections', '/camera0/aruco_detections'),
                ('color_detections', '/camera0/color_detections')
            ]
        ),
        # Camera 1
        Node(
            package='camera_package',
            executable='camera_node',
            name='camera1_node',
            parameters=[{'camera_id': 2}],
            remappings=[('image_raw', '/camera1/image_raw')]
        ),
        Node(
            package='camera_package',
            executable='processing_node',
            name='processor1_node',
            remappings=[
                ('image_raw', '/camera1/image_raw'),
                ('image_processed', '/camera1/image_processed'),
                ('aruco_detections', '/camera1/aruco_detections'),
            ]
        ),
    ])