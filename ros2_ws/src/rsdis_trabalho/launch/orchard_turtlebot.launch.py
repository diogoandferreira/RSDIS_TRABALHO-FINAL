import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rsdis_trabalho = get_package_share_directory('rsdis_trabalho')
    pkg_turtlebot3_gz = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_desc = get_package_share_directory('turtlebot3_description')

    world_path = os.path.join(pkg_rsdis_trabalho, 'worlds', 'orchard.sdf')

    # URDF para o robot_state_publisher (TF)
    urdf_path = os.path.join(pkg_turtlebot3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Modelo SDF do TurtleBot3 Waffle (com DiffDrive)
    tb3_model_path = os.path.join(pkg_turtlebot3_gz, 'models', 'turtlebot3_waffle', 'model.sdf')

    # Meshes do pomar (contem a pasta orchard/orchard_world.dae)
    clearpath_meshes = ('/home/diogo-ferreira/MEEC/1ANO/RSDIS/RSDIS/ros2_ws/src/'
                        'clearpath_simulator/clearpath_gz/meshes')

    set_gz_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'{pkg_turtlebot3_gz}/models:{clearpath_meshes}'
    )

    # 1. Gazebo Sim com o pomar
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    # 2. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # 3. Spawn do TurtleBot3
    spawn_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'turtlebot3_waffle',
                   '-file', tb3_model_path,
                   '-x', '0.0', '-y', '0.0', '-z', '0.3'],
        output='screen',
    )

    # 4. Bridge ROS 2 <-> Gazebo (topicos curtos, definidos no SDF/modelo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/uav/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/uav/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/uav/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[('/uav/camera/image_raw', '/camera/image_raw')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        set_gz_path,
        gz_sim,
        robot_state_publisher,
        spawn_turtlebot,
        bridge,
    ])