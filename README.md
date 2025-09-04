# KITTI to ROS2 Bag Converter

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A ROS 2 package that merges the KITTI raw and odometry datasets into a single ROS 2 bag file. Currently, it combines IMU data from the raw dataset with point clouds and accurate ground truth odometry from the odometry dataset, providing a unified and synchronized dataset for robotics and autonomous driving research.


## Features

- **KITTI Odometry Support**: Convert KITTI odometry sequences with ground truth poses
- **Complete Sensor Data**: Velodyne point clouds, IMU data from raw datasets, and ground truth trajectories
- **ROS2 Native**: Full ROS2 bag format with proper QoS settings
- **TF Integration**: Automatic coordinate frame transforms between sensors
- **Simple setup**: Launch with arguments; no YAML config required

## Available Topics

The generated ROS2 bags contain the following topics:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/kitti/velo/pointcloud` | `sensor_msgs/msg/PointCloud2` | Velodyne LiDAR point clouds |
| `/kitti/oxts/imu` | `sensor_msgs/msg/Imu` | IMU sensor data from OXTS system |
| `/kitti/gtruth/odom` | `nav_msgs/msg/Odometry` | Ground truth odometry poses |
| `/kitti/gtruth/path` | `nav_msgs/msg/Path` | Ground truth trajectory path |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic coordinate transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static coordinate transforms |

## Prerequisites

- **Operating System**: Ubuntu 20.04+ (tested on Ubuntu 24.04)
- **ROS2 Distribution**: Humble, Iron, or Jazzy
- **Python**: 3.8+
- **Dependencies**: 
  - `pykitti` for KITTI dataset handling
  - `opencv-python` for image processing
  - `numpy`, `scipy` for numerical computations
  
## Tested Platforms

- **Ubuntu** 24.04
- **ROS 2** Iron (and newer)
- **Python** 3.8+

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/distancetech/kitti_to_ros2bag.git
```

### 2. Install Python Dependencies

```bash
pip3 install pykitti opencv-python numpy scipy tf-transformations
```

### 3. Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/<your-ros-distro>/setup.bash
colcon build --packages-select kitti_to_ros2bag --symlink-install
source install/setup.bash
```

## Usage

### Dataset Preparation

You need both KITTI odometry data and corresponding raw data to get IMU information.

#### 1. Download KITTI Odometry Dataset

Visit [KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php):
- Download **Velodyne point clouds** (29 GB): `data_odometry_velodyne.zip`
- Download **Ground truth poses** (4 MB): `data_odometry_poses.zip`

Extract and organize as follow:

Your directory structure should look like:
```
~/kitti_data/dataset/
├── poses/
│   ├── 00.txt
│   ├── 01.txt
│   └── ... (ground truth poses)
└── sequences/
    ├── 00/
    │   ├── calib.txt
    │   └── velodyne/
    │       ├── 000000.bin
    │       └── ...
    └── ...
```

#### 2. Download Corresponding Raw Data (for IMU)

For each sequence you want to convert, download the corresponding raw dataset from [KITTI Raw Data](https://www.cvlibs.net/datasets/kitti/raw_data.php).

For example, for sequence 00:
```bash
cd ~/kitti_raw_data
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
unzip 2011_10_03_drive_0027_sync.zip
```

### Available KITTI Sequences

| Sequence | Name | Frames | Raw Dataset Required |
|----------|------|--------|---------------------|
| 00 | 2011_10_03_drive_0027 | 4541 | 2011_10_03_drive_0027_sync.zip |
| 01 | 2011_10_03_drive_0042 | 1101 | 2011_10_03_drive_0042_sync.zip |
| 02 | 2011_10_03_drive_0034 | 4661 | 2011_10_03_drive_0034_sync.zip |
| 03 | 2011_09_26_drive_0067 | 801 | 2011_09_26_drive_0067_sync.zip |
| 04 | 2011_09_30_drive_0016 | 271 | 2011_09_30_drive_0016_sync.zip |
| 05 | 2011_09_30_drive_0018 | 2761 | 2011_09_30_drive_0018_sync.zip |
| 06 | 2011_09_30_drive_0020 | 1101 | 2011_09_30_drive_0020_sync.zip |
| 07 | 2011_09_30_drive_0027 | 1101 | 2011_09_30_drive_0027_sync.zip |
| 08 | 2011_09_30_drive_0028 | 4071 | 2011_09_30_drive_0028_sync.zip |
| 09 | 2011_09_30_drive_0033 | 1591 | 2011_09_30_drive_0033_sync.zip |
| 10 | 2011_09_30_drive_0034 | 1201 | 2011_09_30_drive_0034_sync.zip |

### Launch Arguments

- **data_dir** (required): Path to the KITTI odometry root containing `sequences/` and `poses/`.
- **raw_dir** (optional): Path to the KITTI raw root (date folders). If provided, IMU/GPS will be included.
- **frame_id** (required): KITTI sequence index. Raw IMU/GPS mapping is supported for sequences `00`–`10`.
- **bag_dir** (optional): Output rosbag directory. Defaults to `./odom_bag_XX` where `XX` is the zero-padded `frame_id`.

### Running the Converter

- Odometry only:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch kitti_to_ros2bag kitti_odometry_bag.launch.py \
  data_dir:=/absolute/path/to/kitti_odometry \
  frame_id:=2
```

- Odometry + raw (IMU/GPS) with custom output path:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch kitti_to_ros2bag kitti_odometry_bag.launch.py \
  data_dir:=/absolute/path/to/kitti_odometry \
  raw_dir:=/absolute/path/to/kitti_raw \
  frame_id:=2 \
  bag_dir:=/absolute/path/to/output/my_bag
```

### Verification

Check the generated bag file:
```bash
ros2 bag info /absolute/path/to/output/my_bag
```

## Playing ROS2 Bags

To play the generated bags with proper QoS settings:

```bash
ros2 bag play /absolute/path/to/output/my_bag \
  --qos-profile-overrides-path ~/ros2_ws/src/kitti_to_ros2bag/config/qos_override.yaml
```

## Configuration Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `data_dir` | string | Path to KITTI odometry dataset root (`sequences/` and `poses/`) | - (required) |
| `raw_dir` | string | Path to KITTI raw root (date folders). Enables IMU/GPS if set | '' |
| `frame_id` | int | KITTI sequence index (e.g., 0..10 supported for raw mapping) | 0 |
| `bag_dir` | string | Output bag directory | `./odom_bag_XX` |

### Example Subscriber Nodes

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class KittiImuSubscriber(Node):
    def __init__(self):
        super().__init__('kitti_imu_subscriber')
        
        self.subscription = self.create_subscription(
            Imu,
            '/kitti/oxts/imu',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('KITTI IMU subscriber started')
    
    def imu_callback(self, msg):
        # Extract IMU data
        linear_accel = msg.linear_acceleration
        angular_vel = msg.angular_velocity
        
        # Calculate acceleration magnitude
        accel_magnitude = np.sqrt(
            linear_accel.x**2 + linear_accel.y**2 + linear_accel.z**2
        )
        
        self.get_logger().info(
            f'IMU - Accel: [{linear_accel.x:.3f}, {linear_accel.y:.3f}, {linear_accel.z:.3f}] '
            f'Magnitude: {accel_magnitude:.3f} m/s²'
        )

def main():
    rclpy.init()
    node = KittiImuSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class KittiPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('kitti_pointcloud_subscriber')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/kitti/velo/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info('KITTI Point Cloud subscriber started')
    
    def pointcloud_callback(self, msg):
        # Convert to numpy array
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_array = np.array(points)
        
        if len(points_array) > 0:
            # Calculate statistics
            x_coords = points_array[:, 0]
            y_coords = points_array[:, 1] 
            z_coords = points_array[:, 2]
            
            self.get_logger().info(
                f'Point Cloud - Count: {len(points_array)}, '
                f'X: [{x_coords.min():.2f}, {x_coords.max():.2f}], '
                f'Y: [{y_coords.min():.2f}, {y_coords.max():.2f}], '
                f'Z: [{z_coords.min():.2f}, {z_coords.max():.2f}]'
            )

def main():
    rclpy.init()
    node = KittiPointCloudSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion

class KittiOdometrySubscriber(Node):
    def __init__(self):
        super().__init__('kitti_odometry_subscriber')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/kitti/gtruth/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('KITTI Odometry subscriber started')
    
    def odom_callback(self, msg):
        # Extract position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to euler angles
        euler = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        roll, pitch, yaw = euler
        
        # Extract velocities
        linear_vel = msg.twist.twist.linear
        speed = np.sqrt(linear_vel.x**2 + linear_vel.y**2 + linear_vel.z**2)
        
        self.get_logger().info(
            f'Pose - Position: [{position.x:.2f}, {position.y:.2f}, {position.z:.2f}], '
            f'Yaw: {np.degrees(yaw):.1f}°, Speed: {speed:.2f} m/s'
        )

def main():
    rclpy.init()
    node = KittiOdometrySubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Based on the original [kitti2bag](https://github.com/tomas789/kitti2bag) project
- KITTI dataset provided by the Karlsruhe Institute of Technology and Toyota Technological Institute
- Built for the ROS2 ecosystem

## Support

If you find this package useful, please consider starring the repository. For issues and feature requests, please use the GitHub issue tracker.
