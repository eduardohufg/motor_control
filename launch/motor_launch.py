#Packages to get the address of the YAML file
import os
from ament_index_python.packages import get_package_share_directory

#Launch Pacckages
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #Get the address of the YAML File
    config = os.path.join(get_package_share_directory('motor_control'),'config','config.yaml')

    motor_node = Node(name='dc_motor',
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config],
                       )
    
    sp_node = Node(name='set_point',
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config],
                       )
    
    controller_node = Node(name='controller',
                           package='motor_control',
                           executable='controller',
                           emulate_tty=True,
                           output='screen',
                           parameters=[config],
                           )
    
    graph_node = Node(name='rqt_graph',
                      package='rqt_graph',
                      executable='rqt_graph',
                      output='screen',
                      )
    
    plot_node = Node(name='rqt_plot',
                      package='rqt_plot',
                      executable='rqt_plot',
                      output='screen',
                      arguments=['/dc_motor/motor_speed', 
                                 '/set_point/set_point', 
                                 '/controller/control_input'],
                      )
    
    l_d = LaunchDescription([motor_node, sp_node, controller_node, graph_node, plot_node])

    return l_d