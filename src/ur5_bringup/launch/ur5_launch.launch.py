from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    declared_arguments = []

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5',
        description='type of an UR robot')
    declared_arguments.append(ur_type_arg)
    
    ur_description_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        description='Path to the urdf of UR5'   
    )
    declared_arguments.append(ur_description_arg)
    
    safety_limits_arg = DeclareLaunchArgument(
        'safety_limits',
        default_value='true',
        description="Enables the safety limits controller if true.",
    )
    declared_arguments.append(safety_limits_arg)

    safety_pos_arg = DeclareLaunchArgument(
        'safety_pos_margin',
        default_value='0.15',
        description="The margin to lower and upper limits in the safety controller.",
    )
    declared_arguments.append(safety_pos_arg)

    safety_k_position_arg = DeclareLaunchArgument(
        'safety_k_position',
        default_value='20',
        description="k-position factor in the safety controller.",
    )
    declared_arguments.append(safety_k_position_arg)

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='""',
        description="Prefix of the joint names, useful for "
        "multi-robot setup. If changed than also joint names in the controllers' configuration "
        "have to be updated.",
    )
    declared_arguments.append(tf_prefix_arg)

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('ur5_bringup'), 'rviz', 'rviz_default.rviz']),
        description="RViz config file (absolute path) to use when launching rviz.",
    )
    declared_arguments.append(rviz_config_arg)

    ur_type = LaunchConfiguration("ur_type")
    urdf_path = LaunchConfiguration("urdf_path")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config = LaunchConfiguration("rviz_config")

    ur5_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            urdf_path,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur5",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix

        ]
    )
    ur5_description = {
        "robot_description": ParameterValue(value=ur5_description_content, value_type=str)
    }
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        parameters=[ur5_description, {'use_sim_time': True}],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_node",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
        output="screen"
    )

    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]


    return LaunchDescription(declared_arguments + nodes_to_start)