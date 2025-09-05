
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

package_name = 'kitti_to_ros2bag'

def generate_launch_description():

    odometry_dir_arg = DeclareLaunchArgument(
        'odometry_dir',
        default_value='',
        description='Path to the KITTI dataset root (with sequences/...). Also used as odom_dir.'
    )

    raw_dir_arg = DeclareLaunchArgument(
        'raw_dir',
        default_value='',
        description='Path to the KITTI raw dataset root (with date folders). Optional; leave empty to skip raw/IMU/GPS.'
    )

    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='00',
        description='KITTI odometry sequence index (e.g. 0..10).'
    )

    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value='',
        description='Output rosbag directory. If empty, defaults to ./odom_bag_XX (XX=sequence).'
    )

    kitti_rec = Node(
        package=package_name,
        namespace='',
        executable='kitti_odometry_2_bag_node',
        name='KittiOdom2Bag',
        parameters=[{
            'odometry_dir': LaunchConfiguration('odometry_dir'),
            'raw_dir': LaunchConfiguration('raw_dir'),
            'sequence': ParameterValue(LaunchConfiguration('sequence'), value_type=str),
            'bag_dir': LaunchConfiguration('bag_dir'),
        }],
        output='screen'
    )

    return LaunchDescription([
        odometry_dir_arg,
        raw_dir_arg,
        sequence_arg,
        bag_dir_arg,
        kitti_rec,
    ])
