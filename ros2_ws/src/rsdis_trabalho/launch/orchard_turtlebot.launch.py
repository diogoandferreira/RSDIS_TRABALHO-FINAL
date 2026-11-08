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
    tb3_model_path = os.path.join(
        pkg_turtlebot3_gz, 'models', 'turtlebot3_waffle', 'model.sdf')

    # ---- Caminhos de recursos do Gazebo (têm de vir ANTES do gz_sim) ----
    # models -> model://x3_uav
    set_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_rsdis_trabalho, 'models'))
    # meshes -> model://orchard/orchard_world.dae
    set_meshes = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_rsdis_trabalho, 'meshes'))
    # modelos do TurtleBot3
    set_tb3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_turtlebot3_gz, 'models'))

    # 1. Gazebo Sim com o pomar (o X3 vem do <include> no orchard.sdf)
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
                   '-x', '0.0', '-y', '1.0', '-z', '0.3'],
        output='screen',
    )

    # 4. Bridge ROS 2 <-> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # TurtleBot3
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camara do UAV (usada pelos nós da equipa)
            '/uav/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/uav/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # X3 UAV
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
            '/X3/gazebo/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        set_models,
        set_meshes,
        set_tb3,
        gz_sim,
        robot_state_publisher,
        spawn_turtlebot,
        bridge,
    ])