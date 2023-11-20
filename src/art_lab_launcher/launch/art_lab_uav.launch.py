from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    camera = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense_ros2_camera'), 'launch'),
         '/ros2_intel_realsense.launch.py'])
      ) 

    #px4 = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('px4_ros_com'), 'launch'),
    #     '/sensor_combined_listener.launch.py'])
    #  ) 
    
    compressed_image = Node(
                   package="image_processing",
                   executable="compressed_image"
               )
    
    web_socket = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rosbridge_server'), 'launch'),
         '/rosbridge_websocket_launch.xml'])
      ) 
      
    uav_control = Node(
    		  package="s500_uav_ros2", 
    		  executable="uav_auto",
    		  emulate_tty=True,
    		  output="screen"
    		  )
    
    return LaunchDescription([ 
        camera,
        #px4,
        compressed_image,
        web_socket,
        uav_control
    ])
