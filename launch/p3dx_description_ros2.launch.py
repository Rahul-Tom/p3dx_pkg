import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='p3dx_pkg').find('p3dx_pkg')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_default_view.rviz')
    urdf_file_name = 'pioneer3dx_fixed_joints.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('p3dx_pkg'), 'urdf', urdf_file_name
        )
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    arg_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    arg_tf_prefix = DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='TF2 prefix'
    )

    def get_prefix(context):
        prefix = context.launch_configurations['tf_prefix']
        if (prefix != ''):
            frame_prefix = prefix + '/'
        else:
            frame_prefix = ''
        return [SetLaunchConfiguration('frame_prefix', frame_prefix)]

    get_prefix_fn = OpaqueFunction(function = get_prefix)


    state_publisher = Node(
        namespace=[LaunchConfiguration('tf_prefix')],
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='p3dx_description',
        output='screen',
        parameters=[
            {
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
                'frame_prefix': [LaunchConfiguration('frame_prefix')]
            }
        ]
    )
##Robot localization node
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    ##Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    ##Laser scan Matcher odometry
    odometry_node = Node(
        package='ros2_laser_scan_matcher',
        parameters=[{
                'base_frame': 'base_link',
                'odom_frame': 'odom_matcher',
                'laser_frame': 'laser',
                'publish_odom': 'laser_scan_matcher/odom',
                'publish_tf': False
            }],
        executable='laser_scan_matcher',
        name='odom_laser_scan_match_pub',
    )
  
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
                                                                                
        # arg_sim_time,
        # arg_tf_prefix,
        # get_prefix_fn,
        # state_publisher,
        robot_localization_node,
        odometry_node,
        rviz_node,
        #joy_node
    ])
