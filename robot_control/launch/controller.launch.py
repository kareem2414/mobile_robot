from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"

    )

    wheel_seperation_arg = DeclareLaunchArgument(
        "wheel_seperation",
        default_value="0.17"
        
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_seperation = LaunchConfiguration("wheel_seperation")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller_cpp = Node(
        package="robot_control",
        executable="simple_controller",
        parameters=[{"wheel_seperation":wheel_seperation,
                     "wheel_radius":wheel_radius}]
    )
    return LaunchDescription([
        wheel_radius_arg,
        wheel_seperation_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_cpp

    ])