
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

package_name = 'kitti_to_ros2bag'

def generate_launch_description():


    params_config = os.path.join(
        get_package_share_directory(package_name),
        'kitti_config.yaml'
    )
    kitti_rec = Node(
        package=package_name,
        namespace='',
        executable='kitti_odometry_2_bag_node',
        name='KittiOdom2Bag',
        parameters=[params_config],
        output='screen'
    )

    return LaunchDescription([
        kitti_rec,
    ])
