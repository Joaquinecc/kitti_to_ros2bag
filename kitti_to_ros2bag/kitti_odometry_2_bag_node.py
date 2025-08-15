#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import rosbag2_py
import os
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField,Imu,Image, CameraInfo
from std_msgs.msg import Header # We need Header for PointCloud2 message
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from kitti_to_ros2bag.utils.kitti_utils import KITTIOdometryDataset
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message
from tf2_ros import TransformBroadcaster # ADDED: Import TransformBroadcaster
from tf2_msgs.msg import TFMessage # ADDED: Needed for /tf topic
from kitti_to_ros2bag.utils.utils import quaternion_from_matrix
# ,quaternion_from_euler
from rclpy.time import Time as RospyTime # Import rclpy.time.Time for proper ROS Time objects
from scipy.linalg import inv
import traceback  # Optional, for detailed error logging
import pykitti
from tf_transformations import quaternion_from_euler

VELO_TOPIC_NAME='/kitti/velo/pointcloud'
IMU_TOPIC_NAME='/kitti/oxts/imu'
ODOM_TOPIC_NAME='/kitti/gtruth/odom'
PATH_TOPIC_NAME='/kitti/gtruth/path'
class KittiOdom2Bag(Node):
    def __init__(self):
        """
        Initializes the KittiOdom2Bag node.

        This sets up parameters, dataset loading, bag writer, topic creation, and TF transforms.
        
        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        super().__init__("KittiOdom2Bag")

        # parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sequence', rclpy.Parameter.Type.INTEGER),
                ('data_dir', rclpy.Parameter.Type.STRING),
                ('odom', rclpy.Parameter.Type.BOOL),
                ('odom_dir', rclpy.Parameter.Type.STRING),
                ('bag_dir', rclpy.Parameter.Type.STRING),
                ('raw_data.dir', rclpy.Parameter.Type.STRING),
                ('raw_data.start_frame', rclpy.Parameter.Type.INTEGER),
                ('raw_data.end_frame', rclpy.Parameter.Type.INTEGER),
                ('raw_data.date', rclpy.Parameter.Type.STRING),
                ('raw_data.drive', rclpy.Parameter.Type.STRING),
            ]
        )

        sequence = self.get_parameter('sequence').value
        data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        odom = self.get_parameter('odom').value
        bag_dir = self.get_parameter('bag_dir').get_parameter_value().string_value
        if odom == True:
            odom_dir = self.get_parameter('odom_dir').get_parameter_value().string_value
        else:
            odom_dir = None


        
        self.kitti_dataset = KITTIOdometryDataset(data_dir, sequence, odom_dir)
        self.bridge = CvBridge()
        self.counter = 0
        self.counter_limit = len(self.kitti_dataset.left_images())
        
        self.left_imgs = self.kitti_dataset.left_images()
        self.right_imgs = self.kitti_dataset.right_images()
        self.velodynes_file = self.kitti_dataset.velodyne_plc()
        self.times_file = self.kitti_dataset.times_file()
        self.odom = odom
        # Access raw_data nested fields via flattened keys
        raw_data_dir = self.get_parameter('raw_data.dir').value
        self.kitti_raw = None
        if raw_data_dir:
            try:
                start_frame = self.get_parameter('raw_data.start_frame').value
                end_frame = self.get_parameter('raw_data.end_frame').value
                if end_frame == -1:
                    end_frame = len(self.times_file)
                frames = list(range(start_frame, end_frame))

                date = self.get_parameter('raw_data.date').value
                drive = self.get_parameter('raw_data.drive').value

                self.get_logger().info(f"Loading KITTI raw data: {date} {drive} from {raw_data_dir}, frames {start_frame} to {end_frame}")
                self.kitti_raw = pykitti.raw(raw_data_dir, date, drive, frames=frames)

            except Exception as e:
                self.get_logger().error(f"Failed to load KITTI raw data: {e}")
                self.get_logger().debug(traceback.format_exc())
                rclpy.shutdown()
                return
            
        if odom == True:
            try:
                self.ground_truth = self.kitti_dataset.odom_pose()
            except FileNotFoundError as filenotfounderror:
                self.get_logger().error("Error: {}".format(filenotfounderror))
                rclpy.shutdown()
                return

        # rosbag writer
        self.writer = rosbag2_py.SequentialWriter()
        if os.path.exists(bag_dir):
            self.get_logger().info(f'The directory {bag_dir} already exists. Shutting down...')
            rclpy.shutdown()
        else:
            storage_options = rosbag2_py._storage.StorageOptions(uri=bag_dir, storage_id='sqlite3')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)


        self.p_msg = Path()
        self.p_msg.header.frame_id = "odom" # Initialize path header frame_id

        self.create_topics() # Create topics for the bag file

        self.publish_tf_static()    
   

    def create_topics(self):
        """
        Creates ROS 2 topics for publishing KITTI data.

        Topics include grayscale stereo images, camera info, odometry, path, 
        Velodyne point clouds, and TF frames.

        Returns
        -------
        None
        """
        # left_img_topic_info = rosbag2_py._storage.TopicMetadata(id=510,name='/kitti/camera_gray_left/image', type='sensor_msgs/msg/Image', serialization_format='cdr')
        # right_img_topic_info = rosbag2_py._storage.TopicMetadata(id=511,name='/kitti/camera_gray_right/image', type='sensor_msgs/msg/Image', serialization_format='cdr')
        # left_cam_topic_info = rosbag2_py._storage.TopicMetadata(id=514,name='/kitti/camera_gray_left/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')
        # right_cam_topic_info = rosbag2_py._storage.TopicMetadata(id=515,name='/kitti/camera_gray_right/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')

        odom_topic_info = rosbag2_py._storage.TopicMetadata(id=512,name=ODOM_TOPIC_NAME, type='nav_msgs/msg/Odometry', serialization_format='cdr')
        path_topic_info = rosbag2_py._storage.TopicMetadata(id=513,name=PATH_TOPIC_NAME, type='nav_msgs/msg/Path', serialization_format='cdr')
        velodyne_topic_info = rosbag2_py._storage.TopicMetadata(id=516,name=VELO_TOPIC_NAME, type='sensor_msgs/msg/PointCloud2', serialization_format='cdr')
        tf_static_topic_info = rosbag2_py._storage.TopicMetadata(id=518,name='/tf_static', type='tf2_msgs/msg/TFMessage', serialization_format='cdr')
        tf_topic_info = rosbag2_py._storage.TopicMetadata(id=517,name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr')
        imu_topic= rosbag2_py._storage.TopicMetadata(id=519,name=IMU_TOPIC_NAME,type='sensor_msgs/msg/Imu',serialization_format='cdr',)

        # self.writer.create_topic(left_img_topic_info)
        # self.writer.create_topic(right_img_topic_info)
        # self.writer.create_topic(left_cam_topic_info)
        # self.writer.create_topic(right_cam_topic_info)

        self.writer.create_topic(odom_topic_info)
        self.writer.create_topic(path_topic_info)
        self.writer.create_topic(velodyne_topic_info)
        self.writer.create_topic(tf_topic_info) 
        self.writer.create_topic(tf_static_topic_info) 
        if self.kitti_raw is not None:
            self.writer.create_topic(imu_topic)

        
        
    def process_all_frames(self):
        """
        Iterates through all dataset frames and publishes sensor data.

        For each frame:
        - Reads timestamp
        - Publishes images, odometry, point clouds, and TFs (if enabled)
        - Records all messages to the rosbag

        Returns
        -------
        None
        """

        for counter in range(self.counter_limit):
            time = self.times_file[counter]
            timestamp_ns = int(time * 1e9) # nanoseconds
            
            # self.publish_camera(timestamp_ns, counter)

            if self.odom == True:
                translation = self.ground_truth[counter][:3,3]
                quaternion = quaternion_from_matrix(self.ground_truth[counter])
                self.publish_odom(translation, quaternion, timestamp_ns)
                self.publish_dynamic_tf(translation, quaternion, timestamp_ns)
            if self.kitti_raw is not None:
                self.publish_imu_data(IMU_TOPIC_NAME,timestamp_ns,counter)
            
            #point cloud
            self.publish_velo(VELO_TOPIC_NAME,timestamp_ns,counter)
            self.get_logger().info(f'{counter}-Frames Processed')

        self.get_logger().info('All Frames processed. Stopping...')
        self.writer.close() # Close the bag writer when done

        return 
    def publish_camera(self, timestamp_ns: int, counter: int):
        """
        Publishes left and right grayscale images and their camera info.

        Parameters
        ----------
        timestamp_ns : int
            Timestamp in nanoseconds for the current frame.
        counter : int
            Index of the frame to publish.

        Returns
        -------
        None
        """
        # retrieving images and writing to bag
        left_image = cv2.imread(self.left_imgs[counter])
        right_image = cv2.imread(self.right_imgs[counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='passthrough')
        self.writer.write('/kitti/camera_gray_left/image', serialize_message(left_img_msg), timestamp_ns)
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='passthrough')
        self.writer.write('/kitti/camera_gray_right/image', serialize_message(right_img_msg), timestamp_ns)

        # retrieving project mtx and writing to bag
        p_mtx2 = self.kitti_dataset.projection_matrix(1)
        self.publish_camera_info(p_mtx2, '/kitti/camera_gray_left/camera_info', timestamp_ns)
        p_mtx3 = self.kitti_dataset.projection_matrix(2)
        self.publish_camera_info(p_mtx3, '/kitti/camera_gray_right/camera_info', timestamp_ns)

    def publish_tf_static(self):
        """
        Publishes static TF transforms between base_link, odom, and velo_link frames.

        These transforms are invariant throughout the dataset.

        Returns
        -------
        None
        """
        # self.get_logger().error(f"publish_tf_static {self.kitti_raw.calib}")

        first_timestamp_ns = int(self.times_file[0] * 1e9)
        current_ros_time = RospyTime(nanoseconds=first_timestamp_ns)
        theta = np.deg2rad(90)  # Convert degrees to radians
        #Rotato 90 on the X axis, this just for vizualization purpose, since kitti use different reference axis
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
        tf_global= np.eye(4)
        tf_global[:3,:3]=R_x
        transforms = [
            ('map', 'odom', tf_global),
            ('base_link', 'camera_gray_left', np.eye(4)), #We took the Camera 0 as base_link
            ('base_link', 'velo_link',self.kitti_dataset.calib.T_cam0_velo),  # Invert the transformation for the static TF
            # ('base_link', 'imu_link', self.kitti_raw.calib.T_cam0_imu),   
            # ('camera_gray_left', 'camera_gray_right', self.kitti_dataset.calib.T_cam1_velo),
            # ('camera_gray_left', 'camera_gray_left', self.kitti_dataset.calib.T_cam0_imu),
            # ('camera_gray_left', 'camera_gray_right', self.kitti_dataset.calib.T_cam1_imu),
        ]
     
        # Only add these transforms if kitti_raw exists and has calib
        if hasattr(self, 'kitti_raw') and hasattr(self.kitti_raw, 'calib'):
            transforms.append(('base_link', 'imu_link', self.kitti_raw.calib.T_cam0_imu))
        tfm_static = TFMessage()

        for parent_frame, child_frame, transform in transforms:
            t = transform[0:3, 3]
            q = quaternion_from_matrix(transform)

            tf_msg = TransformStamped()
            tf_msg.header.stamp.sec = first_timestamp_ns // 1_000_000_000
            tf_msg.header.stamp.nanosec = first_timestamp_ns % 1_000_000_000
            tf_msg.header.frame_id = parent_frame
            tf_msg.child_frame_id = child_frame
            
            # Set translation
            tf_msg.transform.translation.x = t[0]
            tf_msg.transform.translation.y = t[1]
            tf_msg.transform.translation.z = t[2]
            tf_msg.transform.rotation.x = q[0]
            tf_msg.transform.rotation.y = q[1]
            tf_msg.transform.rotation.z = q[2]
            tf_msg.transform.rotation.w = q[3]
            # self.tf_broadcaster.sendTransform(tf_msg)
            tfm_static.transforms.append(tf_msg)
            
        self.writer.write('/tf_static', serialize_message(tfm_static),  current_ros_time.nanoseconds)

    def publish_imu_data(self, topic: str, timestamp_ns: int, counter: int):
        """
        Publishes IMU data from the OXTS packet.

        Parameters
        ----------
        oxts : pykitti.raw.oxts.OxtsData
            The OXTS data packet containing IMU readings.
        current_time : datetime
            The current timestamp of the data.

        Returns
        -------
        None
        """
        if counter>= len(self.kitti_raw.oxts):
            return 
        oxts = self.kitti_raw.oxts[counter]


        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.header.stamp.sec = timestamp_ns // 1_000_000_000
        imu_msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        imu_msg.linear_acceleration.x = oxts.packet.af
        imu_msg.linear_acceleration.y = oxts.packet.al
        imu_msg.linear_acceleration.z = oxts.packet.au
        imu_msg.angular_velocity.x = oxts.packet.wf
        imu_msg.angular_velocity.y = oxts.packet.wl
        imu_msg.angular_velocity.z = oxts.packet.wu

        q = quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)


        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        # Covariance matrices are usually set to 0 if not available
        imu_msg.linear_acceleration_covariance[0] = -1.0 # Unknown covariance
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.orientation_covariance[0] = -1.0
        self.writer.write(topic, serialize_message(imu_msg), timestamp_ns)

    def publish_velo(self, topic: str, timestamp_ns: int, counter: int):
        """
        Publishes Velodyne LiDAR point cloud to the specified topic.

        Parameters
        ----------
        topic : str
            Name of the topic to publish point cloud data.
        timestamp_ns : int
            Timestamp in nanoseconds for the frame.
        counter : int
            Index of the point cloud frame.

        Returns
        -------
        None
        """
        velo_filename=self.velodynes_file[counter]
        frame_id="velo_link"
        try:
            points = np.fromfile(velo_filename, dtype=np.float32).reshape(-1, 4)
        except Exception as e:
            self.get_logger().error(f"Error loading Velodyne file {velo_filename}: {e}")
            (points, timestamp_ns, )


        #Build message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp.sec = timestamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        msg.header.frame_id = frame_id

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        #set data format
        msg.is_bigendian = False # Most common for modern systems
        msg.point_step = 16  # 4 floats * 4 bytes/float = 16 bytes per point
        msg.row_step = msg.point_step * len(points) # Total bytes for one row (all points since height=1)
        msg.height = 1       # For unordered point clouds
        msg.width = len(points) # Number of points
        msg.is_dense = True  
        # Convert the numpy array of points directly to a byte array
        msg.data = points.astype(np.float32).tobytes()

        self.writer.write(topic, serialize_message(msg), timestamp_ns)
        self.get_logger().debug(f"Wrote Velodyne point cloud for frame {counter}")

    def publish_camera_info(self, mtx: np.ndarray, topic: str, timestamp: int):
        """
        Publishes CameraInfo messages using projection matrix from KITTI dataset.

        Parameters
        ----------
        mtx : np.ndarray
            3x4 projection matrix of the camera.
        topic : str
            Topic name for CameraInfo messages.
        timestamp : int
            Timestamp in nanoseconds.

        Returns
        -------
        None
        """
        camera_info_msg_2 = CameraInfo()
        camera_info_msg_2.p = mtx.flatten()
        self.writer.write(topic, serialize_message(camera_info_msg_2), timestamp)   
        return
    def publish_dynamic_tf(self, translation: np.ndarray, quaternion: np.ndarray, timestamp_ns: int):
        """
        Publishes a dynamic transform from the map to base_link frame.

        Parameters
        ----------
        translation : np.ndarray
            3-element array with XYZ position.
        quaternion : np.ndarray
            4-element array with quaternion [x, y, z, w].
        timestamp_ns : int
            Timestamp in nanoseconds.

        Returns
        -------
        None
        """
        t_dynamic = TransformStamped()
        t_dynamic.header.stamp.sec = timestamp_ns // 1_000_000_000
        t_dynamic.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        t_dynamic.header.frame_id = 'odom'
        t_dynamic.child_frame_id = 'base_link'

        t_dynamic.transform.translation.x = translation[0]
        t_dynamic.transform.translation.y = translation[1]
        t_dynamic.transform.translation.z = translation[2]
        t_dynamic.transform.rotation.x = quaternion[0]
        t_dynamic.transform.rotation.y = quaternion[1]
        t_dynamic.transform.rotation.z = quaternion[2]
        t_dynamic.transform.rotation.w = quaternion[3]

        tf_oxts_msg = TFMessage()
        tf_oxts_msg.transforms.append(t_dynamic)
        self.writer.write('/tf', serialize_message(tf_oxts_msg), timestamp_ns)


    def publish_odom(self, translation: np.ndarray, quaternion: np.ndarray, timestamp_ns: int):
        """
        Publishes Odometry and Path messages from ground truth.

        Parameters
        ----------
        translation : np.ndarray
            3-element array representing XYZ position.
        quaternion : np.ndarray
            4-element array with orientation in quaternion format [x, y, z, w].
        timestamp_ns : int
            Timestamp in nanoseconds.

        Returns
        -------
        None
        """
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom" 
        odom_msg.child_frame_id = "camera_gray_left"

        odom_msg.pose.pose.position.x = translation[0] 
        odom_msg.pose.pose.position.y = translation[1] 
        odom_msg.pose.pose.position.z = translation[2]

        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        self.writer.write(ODOM_TOPIC_NAME, serialize_message(odom_msg), timestamp_ns)
        self.publish_path(odom_msg, timestamp_ns)
        return

    def publish_path(self, odom_msg: Odometry, timestamp_ns: int):
        """
        Records a PoseStamped message from an Odometry message to a Path.

        Parameters
        ----------
        odom_msg : nav_msgs.msg.Odometry
            Odometry message used to extract position and orientation.
        timestamp_ns : int
            Timestamp in nanoseconds.

        Returns
        -------
        None
        """

        #GT pose
        pose = PoseStamped()
        pose.header.frame_id = "odom"  # Use the same frame_id as the Path header
        pose.pose = odom_msg.pose.pose

        self.p_msg.poses.append(pose)
        self.writer.write(PATH_TOPIC_NAME, serialize_message(self.p_msg), timestamp_ns)
        return

def main(args=None):
    rclpy.init(args=args)
    node = KittiOdom2Bag()
    try:
        node.process_all_frames() # Call the new processing method directly
    except KeyboardInterrupt:
        pass # Allow clean shutdown on Ctrl+C
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
        rclpy.shutdown()
    finally:
        node.destroy_node() # Clean up the node
        rclpy.shutdown() # Shut down the ROS context

if __name__ == '__main__':
    main()