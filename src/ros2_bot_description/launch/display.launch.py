import os  # to find system paths
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value = os.path.join(get_package_share_directory("ros2_bot_description"), "urdf", "ros2bot.urdf.xacro"), # opens path > opens urdf folder > opens urdf file
        description="Absolute path to robot URDF file"
    ) #createing a arguments for robot description contains name and direcotory of the RDF model we want to vizulise
    # setting the default to get the full path to get the folder of are urdf folder and files if model is not manualy or dynamiclly set so we dont have to re write the path ever time
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")])) 
    
    '''here we instantiate the variable before using it 
    creating the robot_decription as a instance of the parameter value class however because we used 
    Xacro format in or urdf file and not pure URDF convention we instead need to convert the xacro model 
    in to urdf format by first run command  in our launch file after importing the Command class from 
    launch.substitutions package'''

    #creating the this node called robot_state publisher to lauch ros2bot.urdf.xacro automatically for use 
    robot_state_publisher = Node(
        package="robot_state_publisher", # naming the package that I want to launch
        executable="robot_state_publisher", # creating the executable that I want to start
        parameters=[{"robot_description": robot_description}]#tell where the urdf model is located import from launch_ros.parameter_descriptions import ParameterDescription to use a variable of this type
    ) #a object of the Node class
#contains list of components/nodes we want to launch

#joint state gui module for the sliders
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", #naming of the package that I want to launch
        executable="joint_state_publisher_gui", # creating the executable that I want to launch 
    )

#starting rviz node
    rviz_node = Node(
        package="rviz2", # from the package named rviz2 that I want to launch
        executable="rviz2", # creating the executable that I want to start
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("ros2_bot_description"), "rviz", "displayconfig.rviz")] #instead of default rviz display config make it load the display config file set up in or rviz folder
    
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ]) #start in order of execution first to last 