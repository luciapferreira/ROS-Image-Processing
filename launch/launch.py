from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
   camera_type = LaunchConfiguration('camera_type')
    
   return LaunchDescription([
      
    DeclareLaunchArgument(
        'camera_type',
        default_value='test1.mov',
        description='Device ID or video file path'
    ),

    Node(
        package='trsa_lab1',
        executable='camera_driver',
        name='camera_driver',
        output='screen',
        parameters=[
            {'camera_type': camera_type}
        ]
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
