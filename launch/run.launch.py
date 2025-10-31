# launch/lio_sam_full.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. Package share directory & launch arguments
    # ------------------------------------------------------------------
    share_dir = get_package_share_directory('lio_sam')

    # ---- LIO-SAM parameter file ------------------------------------------------
    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Full path to the ROS 2 parameters file to use.'
    )
    # ---- IMU-EKF params (inside lio_sam/config) -----------------------
    imu_ekf_params = LaunchConfiguration('imu_ekf_params')
    declare_imu_ekf_params = DeclareLaunchArgument(
        'imu_ekf_params',
        default_value=os.path.join(share_dir, 'config', 'params_ekf.yaml'),
        description='Full path to the EKF IMU filter parameter file.'
    )


    # ---- URDF / xacro -----------------------------------------------------------
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    robot_description = Command(['xacro', ' ', xacro_path])

    # ---- RViz config ------------------------------------------------------------
    rviz_cfg = os.path.join(share_dir, 'config', 'rviz2.rviz')

    # ---- Merger delay -----------------------------------------------------------
    merger_delay = LaunchConfiguration('merger_delay')
    declare_merger_delay = DeclareLaunchArgument(
        'merger_delay',
        default_value='1.0',
        description='Delay (seconds) before launching scans_merger.'
    )

    # ---- Static lidar TF arguments ----------------------------------------------
    front_x = LaunchConfiguration('front_x')
    back_x  = LaunchConfiguration('back_x')
    declare_front_x = DeclareLaunchArgument('front_x', default_value='0.0',
                                            description='X position of front lidar (m)')
    declare_back_x  = DeclareLaunchArgument('back_x',  default_value='-0.5',
                                            description='X position of back lidar (m)')
    

    # ------------------------------------------------------------------
    # 2. Nodes
    # ------------------------------------------------------------------
    nodes = [
        # ---- LIO-SAM parameters -------------------------------------------------
        declare_params,
        declare_imu_ekf_params,
        declare_front_x,
        declare_back_x,
# ---- Front lidar static transform ---------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_lidar_tf',
            arguments=[
                front_x, '0.0', '0.0',  # translation
                '0.0', '0.0', '0.0',     # rotation (Euler)
                'lidar_link', 'velodyne_points2'
            ],
            output='screen'
        ),

        # ---- Back lidar static transform (180° yaw) -----------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_lidar_tf',
            arguments=[
                '-0.5246246', '0.078469', '-0.131473',
                '3.093', '0.0', '0.0',
                'lidar_link', 'velodyne_points1'
            ],
            output='screen'
        ),
        # ---- Static map → odom --------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'map', 'odom'],
            output='screen'
        ),

        # ---- Robot state publisher ------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # ---- LIO-SAM core nodes --------------------------------------------------
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[params_file],
            output='screen'
        ),

        # ---- RViz ---------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        ),

        # ---- Declare lidar TF arguments -----------------------------------------
        declare_front_x,
        declare_back_x,

        # ---- Merger delay argument -----------------------------------------------
        declare_merger_delay,

        # ---- Scans merger (delayed) ---------------------------------------------
        TimerAction(
            period=merger_delay,
            actions=[
                Node(
                    package='lio_sam',          # <-- change if package name differs
                    executable='lio_sam_scans_merger',
                    name='cloud_merger_node',
                    output='screen',
                    parameters=[params_file]
                )
            ]
        ),
        # ------------------------------------------------------------
        #  IMU filter (EKF) 
        # ------------------------------------------------------------
        Node(
            package='lio_sam',                     # <-- same package
            executable='lio_sam_imu_ekf_node',
            name='imu_ekf_node',
            output='screen',
            parameters=[imu_ekf_params]
        ),
    ]

    return LaunchDescription(nodes)