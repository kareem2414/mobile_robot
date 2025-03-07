from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
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
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"),"launch","gzserver.launch.py"
        )))

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py"
        )))
    
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","mobile_robot","-topic","robot_description"],
        output= "screen"
    )
    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot

    ])