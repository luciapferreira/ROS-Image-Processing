from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
   camera_type = LaunchConfiguration('camera_type')
   
   threshold1 = LaunchConfiguration('threshold1')
   threshold2 = LaunchConfiguration('threshold2')

   max_kernel_length = LaunchConfiguration('max_kernel_length')
   max_kernel_gaussian = LaunchConfiguration('max_kernel_gaussian')
    
   return LaunchDescription([
      
    DeclareLaunchArgument(
        'camera_type',
        default_value='test1.mov',
        description='Device ID or video file path'
    ),

    DeclareLaunchArgument(
        'threshold1',
        default_value='30',
        description='OpenCV Canny Threshold1'
    ),
    DeclareLaunchArgument(
        'threshold2',
        default_value='50',
        description='OpenCV Canny Threshold2'
    ),

    DeclareLaunchArgument(
        'max_kernel_length',
        default_value='2',
        description='OpenCV Kernel Lengh for Dilate/Erode'
    ),
        DeclareLaunchArgument(
        'max_kernel_gaussian',
        default_value='21',
        description='OpenCV Kernel Lengh for Gaussian Blur'
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
        parameters=[
            {'threshold1': threshold1},
            {'threshold2': threshold2},
            {'max_kernel_length': max_kernel_length},
            {'max_kernel_gaussian': max_kernel_gaussian}
        ]
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
