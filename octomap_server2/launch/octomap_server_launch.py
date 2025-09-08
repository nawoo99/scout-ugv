#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # === Input & Frame ===
        DeclareLaunchArgument('input_cloud_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('resolution', default_value='0.1'),  # 1cm resolution - optimal for bag files
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='os_sensor'),

        # === Map Rendering for Dense Output ===
        DeclareLaunchArgument('height_map', default_value='True'),
        DeclareLaunchArgument('colored_map', default_value='True'),
        DeclareLaunchArgument('color_factor', default_value='1.0'),  #  maximum color saturation
        DeclareLaunchArgument('compress_map', default_value='False'),  #  no compression for accuracy
        DeclareLaunchArgument('incremental_2D_projection', default_value='True'),  #  better real-time updates

        # === Sensor Model - Bag File Optimized (Prevents Temporal Scattering) ===
        DeclareLaunchArgument('sensor_model/max_range', default_value='12.0'),  #  reduced range for consistent bag replay
        DeclareLaunchArgument('sensor_model/hit', default_value='0.75'),   #  moderate hit (prevents over-accumulation)
        DeclareLaunchArgument('sensor_model/miss', default_value='0.45'),  #  higher miss (prevents voxel removal)
        DeclareLaunchArgument('sensor_model/min', default_value='0.15'),   #  higher min (stability threshold)
        DeclareLaunchArgument('sensor_model/max', default_value='0.85'),   #  lower max (prevents saturation)

        # === Filtering - Preserve Floor & Ceiling ===
        DeclareLaunchArgument('filter_ground', default_value='False'),     # NEVER filter ground/floor
        DeclareLaunchArgument('filter_speckles', default_value='False'),   #  Keep all fine details
        DeclareLaunchArgument('pointcloud_min_z', default_value='-50.0'),  #  capture deep floors/basements
        DeclareLaunchArgument('pointcloud_max_z', default_value='50.0'),   #  capture high ceilings/structures  
        DeclareLaunchArgument('occupancy_min_z', default_value='-50.0'),   #  include all floor levels
        DeclareLaunchArgument('occupancy_max_z', default_value='50.0'),    #  include all ceiling levels

        # === Ground filter parameters - DISABLED but available ===
        DeclareLaunchArgument('ground_filter/distance', default_value='0.01'),  #  tight tolerance if ever enabled
        DeclareLaunchArgument('ground_filter/angle', default_value='0.05'),     #  very strict angle (preserve floor detail)
        DeclareLaunchArgument('ground_filter/plane_distance', default_value='0.01'),  #  precise plane fitting

        # === Advanced Mapping Parameters - Bag File Optimized ===
        DeclareLaunchArgument('publish_free_space', default_value='False'),  #  disable for stability during bag replay
        DeclareLaunchArgument('latch', default_value='True'),               #  essential for bag files
        DeclareLaunchArgument('track_changes', default_value='False'),      #  disable change tracking for bag files
        DeclareLaunchArgument('listen_changes', default_value='False'),     #  disable external changes
        
        # === Bag File Specific Parameters ===
        DeclareLaunchArgument('full_3d_occupancy', default_value='True'),   #  maintain 3D structure
        DeclareLaunchArgument('project_complete_map', default_value='False'), #  keep full 3D
        DeclareLaunchArgument('update_inner_nodes', default_value='False'),  #  reduce computation during bag replay
        DeclareLaunchArgument('pruning', default_value='True'),             #  enable pruning for memory management
        
        # === Dense Update Parameters - Bag File Stability ===
        DeclareLaunchArgument('max_depth', default_value='13'),             #  reduced depth for bag replay stability  
        DeclareLaunchArgument('prob_hit', default_value='0.75'),            #  moderate hit probability
        DeclareLaunchArgument('prob_miss', default_value='0.45'),           #  balanced miss probability
        DeclareLaunchArgument('thres_min', default_value='0.15'),           #  stability threshold
        DeclareLaunchArgument('thres_max', default_value='0.85'),           #  prevent saturation
        
        # === Bag File Stability Parameters ===
        DeclareLaunchArgument('clamping_thres_min', default_value='0.20'),  #  stronger clamping for bag files
        DeclareLaunchArgument('clamping_thres_max', default_value='0.80'),  #  prevent extreme values
        DeclareLaunchArgument('occupancy_thres', default_value='0.55'),     #  balanced occupancy decision

        # === Colorization ===
        DeclareLaunchArgument('color/r', default_value='0.2'),
        DeclareLaunchArgument('color/g', default_value='0.2'),
        DeclareLaunchArgument('color/b', default_value='1.0'),
        DeclareLaunchArgument('color/a', default_value='1.0'),
        DeclareLaunchArgument('color_free/r', default_value='0.0'),
        DeclareLaunchArgument('color_free/g', default_value='1.0'),
        DeclareLaunchArgument('color_free/b', default_value='0.0'),
        DeclareLaunchArgument('color_free/a', default_value='0.2'),

        Node(
            package='octomap_server2',
            executable='octomap_server',
            output='screen',
            remappings=[('cloud_in', LaunchConfiguration('input_cloud_topic'))],
            parameters=[{
                # === Core Parameters ===
                'resolution': LaunchConfiguration('resolution'),
                'frame_id': LaunchConfiguration('frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                
                # === Rendering ===
                'height_map': LaunchConfiguration('height_map'),
                'colored_map': LaunchConfiguration('colored_map'),
                'color_factor': LaunchConfiguration('color_factor'),
                'compress_map': LaunchConfiguration('compress_map'),
                'incremental_2D_projection': LaunchConfiguration('incremental_2D_projection'),
                
                # === Filtering ===
                'filter_ground': LaunchConfiguration('filter_ground'),
                'filter_speckles': LaunchConfiguration('filter_speckles'),
                'pointcloud_min_z': LaunchConfiguration('pointcloud_min_z'),
                'pointcloud_max_z': LaunchConfiguration('pointcloud_max_z'),
                'occupancy_min_z': LaunchConfiguration('occupancy_min_z'),
                'occupancy_max_z': LaunchConfiguration('occupancy_max_z'),
                
                # === Ground Filter ===
                'ground_filter/distance': LaunchConfiguration('ground_filter/distance'),
                'ground_filter/angle': LaunchConfiguration('ground_filter/angle'),
                'ground_filter/plane_distance': LaunchConfiguration('ground_filter/plane_distance'),
                
                # === Sensor Model ===
                'sensor_model/max_range': LaunchConfiguration('sensor_model/max_range'),
                'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
                'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
                'sensor_model/min': LaunchConfiguration('sensor_model/min'),
                'sensor_model/max': LaunchConfiguration('sensor_model/max'),
                
                # === Advanced Parameters ===
                'publish_free_space': LaunchConfiguration('publish_free_space'),
                'latch': LaunchConfiguration('latch'),
                'track_changes': LaunchConfiguration('track_changes'),
                'listen_changes': LaunchConfiguration('listen_changes'),
                'max_depth': LaunchConfiguration('max_depth'),
                'clamping_thres_min': LaunchConfiguration('clamping_thres_min'),
                'clamping_thres_max': LaunchConfiguration('clamping_thres_max'),
                'occupancy_thres': LaunchConfiguration('occupancy_thres'),
                'full_3d_occupancy': LaunchConfiguration('full_3d_occupancy'),
                'project_complete_map': LaunchConfiguration('project_complete_map'),
                'update_inner_nodes': LaunchConfiguration('update_inner_nodes'),
                'pruning': LaunchConfiguration('pruning'),
                'prob_hit': LaunchConfiguration('prob_hit'),
                'prob_miss': LaunchConfiguration('prob_miss'),
                'thres_min': LaunchConfiguration('thres_min'),
                'thres_max': LaunchConfiguration('thres_max'),
                
                # === Colors ===
                'color/r': LaunchConfiguration('color/r'),
                'color/g': LaunchConfiguration('color/g'),
                'color/b': LaunchConfiguration('color/b'),
                'color/a': LaunchConfiguration('color/a'),
                'color_free/r': LaunchConfiguration('color_free/r'),
                'color_free/g': LaunchConfiguration('color_free/g'),
                'color_free/b': LaunchConfiguration('color_free/b'),
                'color_free/a': LaunchConfiguration('color_free/a'),
            }]
        )
    ])
