from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path  # Used to convert paths to Python Path objects for Gazebo
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription  # Actions for launch files
from ament_index_python.packages import get_package_share_directory  # Function to get package directories
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():
    # Get the path to the 'ros2_bot_description' package
    ros2bot_description_dir = get_package_share_directory('ros2_bot_description')

    # Declare a launch argument 'model' with a default value pointing to the robot's URDF file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(ros2bot_description_dir, "urdf", "ros2bot.urdf.xacro"),  # Path to the URDF file
        description="Absolute path to robot URDF file"
    )
    # This argument allows users to specify a different URDF file if needed without changing the launch file.
    # The default value ensures that the URDF file is loaded from the specified path.

    # Determine the ROS 2 distribution to select the appropriate physics engine
    ros_distro = os.environ["ROS_DISTRO"]
    # If using ROS 2 Humble, use the default physics engine; otherwise, specify the plugin for other distributions
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
    # Note: The physics engine plugin may vary based on the ROS 2 distribution (e.g., Foxy, Rolling).

    # Set the 'GZ_SIM_RESOURCE_PATH' environment variable for Gazebo to locate meshes and URDF files
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(ros2bot_description_dir).parent.resolve())  # Converts the path to a string for Gazebo
        ]
    )
    # This environment variable points Gazebo to the directory containing the robot's resources.

    # Generate the robot description parameter by processing the URDF file with xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),  # Processes the xacro file to generate URDF
        value_type=str
    )

    # Define the 'robot_state_publisher' node to publish the robot's state to ROS 2 topics
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            "use_sim_time": True,  # Use simulation time; set to False when running on a real robot
        }]
    )
    # The 'robot_state_publisher' reads the URDF and publishes the transforms (TF) and joint states.

    # Include the Gazebo launch file to start the Gazebo simulation within this launch file
    gazebo_launch_file = IncludeLaunchDescription(
        # Get the 'gz_sim.launch.py' file from the 'ros_gz_sim' package
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ]),
        # Pass launch arguments to Gazebo
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf ", physics_engine])  # Gazebo verbosity, auto-start, world file, physics engine
        ]
    )
    # '-v 4' sets the verbosity level to 4 for detailed logging.
    # '-r' starts the simulation immediately.
    # 'empty.sdf' specifies the world file to load (an empty world in this case).
    # 'physics_engine' adds the appropriate physics engine argument based on ROS 2 distribution.

    # Define the node to spawn the robot entity into Gazebo
    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',  # The topic where the robot's URDF is published
            '-name', 'ros2_bot'  # The name to assign to the robot in Gazebo
        ],
        output='screen'
    )
    # This node uses the 'create' executable to spawn the robot into the Gazebo simulation.

    # Define the node to bridge Gazebo topics to ROS 2 topics
    gazebo_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'  # Bridge the '/clock' topic for simulation time
        ],
        output='screen'
    )
    # This bridge allows ROS 2 nodes to use the simulation time from Gazebo.

    # Return the LaunchDescription with all the nodes and actions
    return LaunchDescription([
        model_arg,  # Include the model argument
        gazebo_resource_path,  # Set the Gazebo resource path environment variable
        robot_state_publisher,  # Start the robot_state_publisher node
        gazebo_launch_file,  # Include the Gazebo launch file
        gazebo_spawn_entity,  # Spawn the robot in Gazebo
        gazebo_ros2_bridge  # Start the Gazebo-ROS 2 bridge
    ])
