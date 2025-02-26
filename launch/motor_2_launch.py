from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node1 = Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       namespace='group1',
                       )
    
    sp_node1 = Node(name="sp_gen",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       namespace='group1',
                       )
    
    motor_node2 = Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       namespace='group2',
                       )
    
    sp_node2 = Node(name="sp_gen",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       namespace='group2',
                       )
    
    
    
    l_d = LaunchDescription([motor_node1, sp_node1, motor_node2, sp_node2])

    return l_d