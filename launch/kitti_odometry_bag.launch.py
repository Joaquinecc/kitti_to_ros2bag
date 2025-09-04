
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

package_name = 'kitti_to_ros2bag'

def generate_launch_description():

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='',
        description='Path to the KITTI dataset root (with sequences/...). Also used as odom_dir.'
    )

    raw_dir_arg = DeclareLaunchArgument(
        'raw_dir',
        default_value='',
        description='Path to the KITTI raw dataset root (with date folders). Optional; leave empty to skip raw/IMU/GPS.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='0',
        description='KITTI odometry sequence index (e.g. 0..10).'
    )

    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value='',
        description='Output rosbag directory. If empty, defaults to ./odom_bag_XX (XX=frame_id).'
    )

    kitti_rec = Node(
        package=package_name,
        namespace='',
        executable='kitti_odometry_2_bag_node',
        name='KittiOdom2Bag',
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'raw_dir': LaunchConfiguration('raw_dir'),
            'frame_id': LaunchConfiguration('frame_id'),
            'bag_dir': LaunchConfiguration('bag_dir'),
        }],
        output='screen'
    )

    return LaunchDescription([
        data_dir_arg,
        raw_dir_arg,
        frame_id_arg,
        bag_dir_arg,
        kitti_rec,
    ])
