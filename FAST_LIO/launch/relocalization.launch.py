import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
  rviz_use = LaunchConfiguration('rviz')
  robot_tf_use = LaunchConfiguration('robot_tf')

  # icp relocalization
  map_odom_trans = Node(
      package='icp_relocalization',
      executable='transform_publisher',
      name='transform_publisher',
      output='screen'
  )

  icp_node = Node(
      package='icp_relocalization',
      executable='icp_node',
      name='icp_node',
      output='screen',
      parameters=[
          # --- Blue ---
          # {'initial_x':14.16},
          # {'initial_y':5.35},
          # {'initial_z':0.0},
          # {'initial_a':3.14},

          # --- Red ---
          {'initial_x':0.0},
          {'initial_y':0.0},
          {'initial_z':1.2},
          {'initial_a':0.0},

          {'map_voxel_leaf_size':0.5},
          {'cloud_voxel_leaf_size':0.3},
          {'local_map_cube_side_length':12.0}, # <= 0 disables local ICP input cloud crop; set > 0 in meters
          {'local_map_min_points':50},
          {'map_frame_id':'map'},
          {'solver_max_iter':100},
          {'max_correspondence_distance':0.1},
          {'RANSAC_outlier_rejection_threshold':0.5},
          {'map_path':'/workspace/holosoma/src/holosoma/holosoma/data/robots/g1/scenes/2026_5_22.pcd'},
          # {'map_path':'/workspace/holosoma/src/holosoma/holosoma/data/robots/g1/scenes/scene_icp_room_12m.pcd'}, # for local map crop test
          {'fitness_score_thre':0.1}, # 是最近点距离的平均值，越小越严格
          {'converged_count_thre': 20}, # pcl pub at 10 hz, 2s
          {'pcl_type':'livox'},
      ],
  )
  
  # fast-lio localization   
  config = os.path.join(
      get_package_share_directory('fast_lio'), 'config', 'fast_lio_relocalization_param.yaml')
  
  fast_lio_node = Node(
      package='fast_lio',
      executable='fastlio_mapping',
      parameters=[
          config
      ],
      output='screen',
      remappings=[('/Odometry','/state_estimation')]
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('fast_lio'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen',
    condition=IfCondition(rviz_use)
  )

  declare_rviz_cmd = DeclareLaunchArgument(
    'rviz',
    default_value='true',
    description='Start RViz visualization. Disabled by default for high-rate localization.'
  )

  declare_robot_tf_cmd = DeclareLaunchArgument(
    'robot_tf',
    default_value='true',
    description='Start G1 robot_state_publisher so FAST-LIO can resolve IMU-to-pelvis TF.'
  )

  robot_tf_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('holosoma_robot_description'),
        'launch',
        'g1_robot_state_publisher.launch.py'
      )
    ),
    launch_arguments={
      'joint_state_source': 'none',
      'joint_states_topic': '/joint_states',
      'publish_frequency': '200.0',
    }.items(),
    condition=IfCondition(robot_tf_use)
  )

  delayed_start_lio = TimerAction(
    period=5.0,
    actions=[
      icp_node,
      fast_lio_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(declare_rviz_cmd)
  ld.add_action(declare_robot_tf_cmd)
  ld.add_action(robot_tf_launch)
  ld.add_action(map_odom_trans)
  # ld.add_action(icp_node)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_lio)

  return ld
