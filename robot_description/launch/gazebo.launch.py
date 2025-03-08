from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory, get_package_prefix



def generate_launch_description():
    robot_description_path = get_package_share_directory("robot_description")
    robot_description_prefix = get_package_prefix("robot_description")
    model_path = os.path.join(robot_description_path,"models")
    model_path += os.pathsep + os.path.join(robot_description_prefix,"share")

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",model_path)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("robot_description"),"urdf","robot_description.urdf.xacro")
    )
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    world_path = PathJoinSubstitution([
        robot_description_path,
        "worlds",
        PythonExpression(expression=["'",LaunchConfiguration("world_name"), "'", "+'.world'"])
    ])

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
            launch_arguments={"gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])}.items())

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )
    
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","mobile_robot","-topic","robot_description"],
        output= "screen"
    )
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])