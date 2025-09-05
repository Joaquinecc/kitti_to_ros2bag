# KITTI to ROS2 Bag Converter

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A ROS 2 package to create ROS 2 bag files from the KITTI datasets. It uses KITTI Odometry for ground-truth poses, Velodyne point clouds, and camera images, and optionally integrates KITTI Raw to include IMU and GPS. If no KITTI Raw directory is provided, only odometry-derived data (odometry, TF, path, point clouds, cameras, camera info) are recorded.


## Available Topics

All bags include these topics:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/kitti/velo/pointcloud` | `sensor_msgs/msg/PointCloud2` | Velodyne LiDAR point clouds |
| `/kitti/gtruth/odom` | `nav_msgs/msg/Odometry` | Ground-truth odometry poses |
| `/kitti/gtruth/path` | `nav_msgs/msg/Path` | Accumulated ground-truth trajectory |
| `/kitti/camera_gray_left/image` | `sensor_msgs/msg/Image` | Grayscale left camera |
| `/kitti/camera_gray_right/image` | `sensor_msgs/msg/Image` | Grayscale right camera |
| `/kitti/camera_color_left/image` | `sensor_msgs/msg/Image` | Color left camera |
| `/kitti/camera_color_right/image` | `sensor_msgs/msg/Image` | Color right camera |
| `/kitti/camera_gray_left/camera_info` | `sensor_msgs/msg/CameraInfo` | Grayscale left calibration |
| `/kitti/camera_gray_right/camera_info` | `sensor_msgs/msg/CameraInfo` | Grayscale right calibration |
| `/kitti/camera_color_left/camera_info` | `sensor_msgs/msg/CameraInfo` | Color left calibration |
| `/kitti/camera_color_right/camera_info` | `sensor_msgs/msg/CameraInfo` | Color right calibration |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms (`odom` → `base_link`) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms (map/odom/sensors) |

If KITTI Raw is provided and supported for the sequence, these are added:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/kitti/oxts/imu` | `sensor_msgs/msg/Imu` | IMU data from OXTS |
| `/kitti/oxts/gps` | `sensor_msgs/msg/NavSatFix` | GPS fix from OXTS |

## Prerequisites

- **Operating System**: Ubuntu 20.04+ (tested on Ubuntu 24.04)
- **ROS 2 Distribution**: Humble, Iron, or Jazzy
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

- KITTI Odometry is required
- KITTI Raw is optional (only if you want IMU/GPS); mapping is automatic for supported sequences

#### 1. KITTI Odometry Dataset

Download from the KITTI Odometry Benchmark (poses and velodyne). Directory structure:
```
~/kitti_data/dataset/
├── poses/
│   ├── 00.txt
│   ├── 01.txt
│   └── ...
└── sequences/
    ├── 00/
    │   ├── calib.txt
    │   ├── image_0/  (for images if available)
    │   ├── image_1/
    │   ├── image_2/
    │   ├── image_3/
    │   └── velodyne/
    └── ...
```

#### 2. KITTI Raw Dataset (optional, for IMU/GPS)

Place extracted drives under a root such as `~/kitti_raw_data/2011_10_03/2011_10_03_drive_0027_sync/`.

### Launch Arguments

- `odometry_dir` (required): Path to KITTI odometry root containing `sequences/` and `poses/`.
- `raw_dir` (optional): Path to KITTI Raw root (date folders). If provided, IMU/GPS will be included when available for the sequence.
- `sequence` (required): KITTI sequence string (e.g., `00`, `01`, ...).
- `bag_dir` (optional): Output rosbag directory. Default is `./odom_bag_XX` where `XX` is the sequence.

### Running the Converter

- Odometry only:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch kitti_to_ros2bag kitti_odometry_bag.launch.py \
  odometry_dir:=/absolute/path/to/kitti_odometry \
  sequence:=02
```

- Odometry + Raw (adds IMU/GPS) and custom output path:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch kitti_to_ros2bag kitti_odometry_bag.launch.py \
  odometry_dir:=/absolute/path/to/kitti_odometry \
  raw_dir:=/absolute/path/to/kitti_raw \
  sequence:=02 \
  bag_dir:=/absolute/path/to/output/my_bag
```

### Verification

```bash
ros2 bag info /absolute/path/to/output/my_bag
```

## Playing ROS 2 Bags

To play with proper QoS settings:

```bash
ros2 bag play /absolute/path/to/output/my_bag \
  --qos-profile-overrides-path ~/ros2_ws/src/kitti_to_ros2bag/config/qos_override.yaml
```



## Example Subscriber Nodes

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class KittiImuSubscriber(Node):
    def __init__(self):
        super().__init__('kitti_imu_subscriber')
        self.subscription = self.create_subscription(Imu, '/kitti/oxts/imu', self.imu_callback, 10)
        self.get_logger().info('KITTI IMU subscriber started')

    def imu_callback(self, msg):
        self.get_logger().info(f'IMU accel x: {msg.linear_acceleration.x:.3f}')

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

class KittiPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('kitti_pointcloud_subscriber')
        self.subscription = self.create_subscription(PointCloud2, '/kitti/velo/pointcloud', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'PointCloud2: width={msg.width}, fields={[f.name for f in msg.fields]}')

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

class KittiOdometrySubscriber(Node):
    def __init__(self):
        super().__init__('kitti_odometry_subscriber')
        self.subscription = self.create_subscription(Odometry, '/kitti/gtruth/odom', self.cb, 10)

    def cb(self, msg):
        p = msg.pose.pose.position
        self.get_logger().info(f'Position: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]')

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


## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.


## Support

If you find this package useful, please consider starring the repository. For issues and feature requests, please use the GitHub issue tracker.
