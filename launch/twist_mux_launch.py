from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    twist_mux = os.path.join(get_package_share_directory('p3dx_pkg'),'config','twist_mux.yaml')

    twist_mux = Node(
            package='twist_mux', 
            executable='twist_mux',
            name = 'twist_mux_node',
            parameters=[twist_mux],
            remappings=[('/cmd_vel_out', '/turtle1/cmd_vel')]
            )
#...

# And add to launch description at the bottom

    return LaunchDescription([
        twist_mux     
    ])
