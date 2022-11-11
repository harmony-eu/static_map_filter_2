from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_static_map_filter',
            namespace='',
            executable='laser_static_map_filter',
            name='laser_static_map_filter',
            remappings=[('/map', '/map_inflated'),
                        ('/map_updates', '/map_inflated_updates'),
                        ('/map_server/map', '/map_server_inflated/map'),
                        ('/scan', '/merged_scan_cloud'),
                        ('/scan_filtered', '/scan_filtered_2'),]
        ),
    ])