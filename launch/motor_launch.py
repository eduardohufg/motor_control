#Packages to get the address of the YAML file
import os
from ament_index_python.packages import get_package_share_directory

#Launch Pacckages
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #Get the address of the YAML File
    config = os.path.join(get_package_share_directory('motor_control'),'config','params.yaml')

    motor_node = Node(name="dc_motor",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config],
                       )
    
    sp_node = Node(name="set_point",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config],
                       )
    
    controller_node = Node(name="controller",
                           package='motor_control',
                           executable='controller',
                           emulate_tty=True,
                           output='screen',
                           parameters=[config],
                           )
    
    l_d = LaunchDescription([motor_node, sp_node, controller_node])

    return l_d