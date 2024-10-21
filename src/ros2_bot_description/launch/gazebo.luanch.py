from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable #creating environment variable for linux
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

import os

def generate_launch_description():
    ros2bot_description_dir = get_package_share_directory('ros2_bot_description')

    model_arg = DeclareLaunchArgument(
        name="model", 
    
        default_value = os.path.join(ros2bot_description_dir, "urdf", "ros2bot.urdf.xacro"), # opens path > opens urdf folder > opens urdf file
        description="Absolute path to robot URDF file"
    ) #createing a arguments for robot description contains name and direcotory of the RDF model we want to vizulise
    # setting the default to get the full path to get the folder of are urdf folder and files if model is not manualy or dynamiclly set so we dont have to re write the path ever time
    gazebo_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", 
                                                value=[ros2bot_description_dir])
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")])) 

    robot_state_publisher =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 
                    "use_sim_time": True,}]#use_sim_time  general parameter that is avalible from Node to use simiulation time or can set to fale if using on real physical robot
    )

    
    return LaunchDescription([])