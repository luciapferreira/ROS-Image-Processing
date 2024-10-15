from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
   return LaunchDescription([

    Node(
        package='trsa_lab1',
        executable='camera_driver',
        name='camera_driver',
        output='screen',
    ),
    Node(
        package='trsa_lab1',
        executable='camera_rectifier',
        output='screen',
        name='camera_rectifier'
    ),
    Node(
        package='trsa_lab1',
        executable='image_convert',
        output='screen',
        name='image_convert',
    ),
    Node(
        package='trsa_lab1',
        executable='object_detection',
        output='screen',
        name='object_detection'
    ),
    Node(
        package='trsa_lab1',
        executable='camera_reader',
        output='screen',
        name='camera_reader'
    ),
   ])
