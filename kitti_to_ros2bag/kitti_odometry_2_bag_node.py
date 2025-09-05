#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import rosbag2_py
import os
from typing import List, Optional, Union, Tuple
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu, Image, CameraInfo, NavSatFix, NavSatStatus
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message
from tf2_msgs.msg import TFMessage
from kitti_to_ros2bag.utils.utils import quaternion_from_matrix, sequence_to_raw,quaternion_from_euler
from rclpy.time import Time as RospyTime # Import rclpy.time.Time for proper ROS Time objects
from scipy.linalg import inv
import traceback
import pykitti

VELO_TOPIC_NAME='/kitti/velo/pointcloud'
IMU_TOPIC_NAME='/kitti/oxts/imu'
ODOM_TOPIC_NAME='/kitti/gtruth/odom'
PATH_TOPIC_NAME='/kitti/gtruth/path'
GPS_TOPIC_NAME='/kitti/oxts/gps'
CAM_0_TOPIC_NAME='/kitti/camera_gray_left/image'
CAM_1_TOPIC_NAME='/kitti/camera_gray_right/image'
CAM_0_INFO_TOPIC_NAME='/kitti/camera_gray_left/camera_info'
CAM_1_INFO_TOPIC_NAME='/kitti/camera_gray_right/camera_info'
CAM_2_TOPIC_NAME='/kitti/camera_color_left/image'
CAM_3_TOPIC_NAME='/kitti/camera_color_right/image'
CAM_2_INFO_TOPIC_NAME='/kitti/camera_color_left/camera_info'
CAM_3_INFO_TOPIC_NAME='/kitti/camera_color_right/camera_info'
TF_TOPIC_NAME='/tf'
TF_STATIC_TOPIC_NAME='/tf_static'

class KittiOdom2Bag(Node):
    """
    ROS 2 node for converting KITTI odometry dataset to ROS 2 bag format.
    
    This class handles loading KITTI odometry data, processing sensor information
    including LiDAR point clouds, stereo images, camera calibration, and ground
    truth odometry, and optionally IMU/GPS if KITTI raw data is provided; then
    writes all data to a ROS 2 bag file with proper timestamps and transforms.
    
    Attributes
    ----------
    kitti_dataset : KITTIOdometryDataset
        Handler for KITTI odometry dataset access.
    bridge : CvBridge
        OpenCV-ROS image bridge for image conversions.
    counter : int
        Current frame counter for processing.
    counter_limit : int
        Total number of frames to process.
    writer : rosbag2_py.SequentialWriter
        ROS 2 bag writer for output.
    kitti_raw : Optional[pykitti.raw]
        Raw KITTI data handler for IMU information.
    p_msg : Path
        Accumulated path message for trajectory visualization.
    """

    def __init__(self) -> None:
        """
        Initialize the KittiOdom2Bag node.

        Sets up parameters, loads KITTI dataset, initializes bag writer, 
        creates topics, and publishes static transforms. Handles both 
        odometry and raw KITTI data depending on configuration.
        
        Parameters
        ----------
        None

        Returns
        -------
        None
        
        Raises
        ------
        FileNotFoundError
            If ground truth odometry file is not found.
        Exception
            If KITTI raw data cannot be loaded.
        """
        super().__init__("KittiOdom2Bag")

        # Simplified parameters
        self.declare_parameter('odometry_dir', '')
        self.declare_parameter('raw_dir', '')
        self.declare_parameter('sequence', '00')
        self.declare_parameter('bag_dir', 'output_bag')  # optional; default derived if empty

        sequence = str(self.get_parameter('sequence').value)
        odometry_dir = str(self.get_parameter('odometry_dir').value)
        raw_dir = str(self.get_parameter('raw_dir').value)
        bag_dir = str(self.get_parameter('bag_dir').value)
        self.get_logger().info(f"Parameters loaded: sequence={sequence}, odometry_dir='{odometry_dir}', raw_dir='{raw_dir}', bag_dir='{bag_dir}''")

        if not odometry_dir:
            self.get_logger().error('Parameter odometry_dir is required.')
            rclpy.shutdown()
            return

 

        # Setup dataset
        self.kitti_odometry = pykitti.odometry(odometry_dir, sequence)
        self.counter = 0
        self.counter_limit = len(self.kitti_odometry)
        self.times_file = self.kitti_odometry.timestamps

        self.bridge = CvBridge()

        self.get_logger().info(f"Total number of frames {self.counter_limit}")    

        # Derive raw mapping from sequence if raw_dir provided
        self.kitti_raw = None
        if raw_dir and sequence in sequence_to_raw:
            try:
                date = sequence_to_raw[sequence]['date']
                drive = sequence_to_raw[sequence]['drive']
                start_frame = sequence_to_raw[sequence]['start']
                end_frame = sequence_to_raw[sequence]['end']
                frames = list(range(start_frame, end_frame + 1))
                self.get_logger().info(f"Loading KITTI raw data: {date} {drive} from {raw_dir}, frames {start_frame} to {end_frame}")
                self.kitti_raw = pykitti.raw(raw_dir, date, drive, frames=frames)
            except Exception as e:
                self.get_logger().warn(f"Failed to load KITTI raw data: {e}. IMU/GPS will be disabled.")
                self.get_logger().debug(traceback.format_exc())
        else:
            if not raw_dir:
                self.get_logger().info("No raw_dir provided. IMU/GPS will be disabled.")
            elif sequence not in sequence_to_raw:
                self.get_logger().warn(f"No raw mapping found for sequence {sequence:02d}. IMU/GPS will be disabled.")
            


        # rosbag writer
        self.writer = rosbag2_py.SequentialWriter()
        if not bag_dir:
            bag_dir = os.path.join(os.getcwd(), f'odom_bag_{sequence:02d}')
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
        #publish camera info
        self.publish_camera_info(camera_id=0, topic=CAM_0_INFO_TOPIC_NAME , timestamp=0)
        self.publish_camera_info(camera_id=1, topic=CAM_1_INFO_TOPIC_NAME, timestamp=0)
        self.publish_camera_info(camera_id=2, topic=CAM_2_INFO_TOPIC_NAME, timestamp=0)
        self.publish_camera_info(camera_id=3, topic=CAM_3_INFO_TOPIC_NAME, timestamp=0)

    def create_topics(self) -> None:
        """
        Create ROS 2 topics for publishing KITTI data to bag file.

        Creates topic metadata for odometry, path, Velodyne point clouds,
        TF transforms, and optionally IMU data depending on raw data availability.
        Topics include grayscale stereo images, camera info, odometry, path, 
        Velodyne point clouds, and TF frames.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        #QoS for static topics
        qos_tf_static = rosbag2_py._storage.QoS(1).keep_last(1).reliable().transient_local()

        #Camera 0
        left_img_topic_info = rosbag2_py._storage.TopicMetadata(id=510,name=CAM_0_TOPIC_NAME, type='sensor_msgs/msg/Image', serialization_format='cdr',offered_qos_profiles=[qos_tf_static])
        left_cam_topic_info = rosbag2_py._storage.TopicMetadata(id=514,name=CAM_0_INFO_TOPIC_NAME, type='sensor_msgs/msg/CameraInfo', 
        serialization_format='cdr',offered_qos_profiles=[qos_tf_static])

        #Camera 1
        right_img_topic_info = rosbag2_py._storage.TopicMetadata(id=511,name=CAM_1_TOPIC_NAME, type='sensor_msgs/msg/Image', serialization_format='cdr',offered_qos_profiles=[qos_tf_static])
        right_cam_topic_info = rosbag2_py._storage.TopicMetadata(id=515,name=CAM_1_INFO_TOPIC_NAME, 
        type='sensor_msgs/msg/CameraInfo', serialization_format='cdr',offered_qos_profiles=[qos_tf_static])

        #Camera 2 (color left)
        color_left_img_topic_info = rosbag2_py._storage.TopicMetadata(id=521,name=CAM_2_TOPIC_NAME, type='sensor_msgs/msg/Image', serialization_format='cdr',offered_qos_profiles=[qos_tf_static])
        color_left_cam_topic_info = rosbag2_py._storage.TopicMetadata(id=523,name=CAM_2_INFO_TOPIC_NAME, type='sensor_msgs/msg/CameraInfo', 
        serialization_format='cdr',offered_qos_profiles=[qos_tf_static])

        #Camera 3 (color right)
        color_right_img_topic_info = rosbag2_py._storage.TopicMetadata(id=522,name=CAM_3_TOPIC_NAME, type='sensor_msgs/msg/Image', serialization_format='cdr',offered_qos_profiles=[qos_tf_static])
        color_right_cam_topic_info = rosbag2_py._storage.TopicMetadata(id=524,name=CAM_3_INFO_TOPIC_NAME, type='sensor_msgs/msg/CameraInfo', 
        serialization_format='cdr',offered_qos_profiles=[qos_tf_static])
        
        odom_topic_info = rosbag2_py._storage.TopicMetadata(id=512,name=ODOM_TOPIC_NAME, type='nav_msgs/msg/Odometry', serialization_format='cdr')
        path_topic_info = rosbag2_py._storage.TopicMetadata(id=513,name=PATH_TOPIC_NAME, type='nav_msgs/msg/Path', serialization_format='cdr')
        velodyne_topic_info = rosbag2_py._storage.TopicMetadata(id=516,name=VELO_TOPIC_NAME, type='sensor_msgs/msg/PointCloud2', serialization_format='cdr')
        imu_topic= rosbag2_py._storage.TopicMetadata(id=519,name=IMU_TOPIC_NAME,type='sensor_msgs/msg/Imu',serialization_format='cdr',)
        gps_topic_info = rosbag2_py._storage.TopicMetadata(id=520,name=GPS_TOPIC_NAME,type='sensor_msgs/msg/NavSatFix',serialization_format='cdr',)
        tf_topic_info = rosbag2_py._storage.TopicMetadata(id=517,name=TF_TOPIC_NAME, type='tf2_msgs/msg/TFMessage', serialization_format='cdr',)
        tf_static_topic_info = rosbag2_py._storage.TopicMetadata(id=518,name=TF_STATIC_TOPIC_NAME, type='tf2_msgs/msg/TFMessage', 
        serialization_format='cdr',offered_qos_profiles=[qos_tf_static])





        self.writer.create_topic(left_img_topic_info)
        self.writer.create_topic(right_img_topic_info)
        self.writer.create_topic(color_left_img_topic_info)
        self.writer.create_topic(color_right_img_topic_info)
        self.writer.create_topic(left_cam_topic_info)
        self.writer.create_topic(right_cam_topic_info)
        self.writer.create_topic(color_left_cam_topic_info)
        self.writer.create_topic(color_right_cam_topic_info)
        self.writer.create_topic(odom_topic_info)
        self.writer.create_topic(path_topic_info)
        self.writer.create_topic(velodyne_topic_info)
        self.writer.create_topic(tf_topic_info) 
        self.writer.create_topic(tf_static_topic_info) 
        if self.kitti_raw is not None:
            self.writer.create_topic(imu_topic)
            self.writer.create_topic(gps_topic_info)

        
    def process_all_frames(self) -> None:
        """
        Process all frames in the KITTI dataset and write to bag file.

        Iterates through all dataset frames sequentially, reading timestamps,
        publishing sensor data (images, odometry, point clouds, IMU), and 
        recording all messages to the rosbag. Handles both raw and odometry
        data depending on configuration.

        For each frame:
        - Reads timestamp from times file
        - Publishes ground truth odometry and path (if available)
        - Publishes dynamic TF transforms
        - Publishes IMU data (if raw data available)
        - Publishes Velodyne point clouds
        - Records all messages to the rosbag

        Parameters
        ----------
        None

        Returns
        -------
        None
        
        Examples
        --------
        >>> node = KittiOdom2Bag()
        >>> node.process_all_frames()  # Processes all frames and writes to bag
        """

        for counter in range(self.counter_limit):
            time = self.kitti_odometry.timestamps[counter]
            timestamp_ns = int(time.total_seconds() * 1e9)

            #Odometry
            translation = self.kitti_odometry.poses[counter][:3,3]
            quaternion = quaternion_from_matrix(self.kitti_odometry.poses[counter])
            self.publish_odom(translation, quaternion, timestamp_ns)
            self.publish_dynamic_tf(translation, quaternion, timestamp_ns)

            if self.kitti_raw is not None:
                self.publish_imu_data(IMU_TOPIC_NAME,timestamp_ns,counter)
                self.publish_gps_data(GPS_TOPIC_NAME, timestamp_ns, counter)

            
            
            # #point cloud
            self.publish_velo(VELO_TOPIC_NAME,timestamp_ns,counter)

            # #Camera
            self.publish_camera(timestamp_ns, counter)

            self.get_logger().info(f'{counter}-Frames Processed')

        self.get_logger().info('All Frames processed. Stopping...')
        self.writer.close() # Close the bag writer when done

        return 
        
    def publish_camera(self, timestamp_ns: int, counter: int) -> None:
        """
        Publish grayscale and color stereo images.

        Reads image pairs from the KITTI odometry dataset (left/right grayscale
        and left/right color), converts them to ROS Image messages, and writes
        them to the bag file. Camera calibration is published separately by
        publish_camera_info().

        Parameters
        ----------
        timestamp_ns : int
            Timestamp in nanoseconds for the current frame.
        counter : int
            Frame index to publish, used to access specific image files.

        Returns
        -------
        None
        
        Examples
        --------
        >>> node = KittiOdom2Bag()
        >>> node.publish_camera(1234567890, 42)  # Publish frame 42 at given timestamp
        """
        #Cam 0
        left_image = cv2.imread(self.kitti_odometry.cam0_files[counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='passthrough')
        #Cam 1
        right_image = cv2.imread(self.kitti_odometry.cam1_files[counter])
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='passthrough')
        #Cam 2 (color left)
        color_left_image = cv2.imread(self.kitti_odometry.cam2_files[counter])
        color_left_img_msg = self.bridge.cv2_to_imgmsg(color_left_image, encoding='passthrough')
        #Cam 3 (color right)
        color_right_image = cv2.imread(self.kitti_odometry.cam3_files[counter])
        color_right_img_msg = self.bridge.cv2_to_imgmsg(color_right_image, encoding='passthrough')



        self.writer.write(CAM_0_TOPIC_NAME, serialize_message(left_img_msg), timestamp_ns)
        self.writer.write(CAM_1_TOPIC_NAME, serialize_message(right_img_msg), timestamp_ns)
        self.writer.write(CAM_2_TOPIC_NAME, serialize_message(color_left_img_msg), timestamp_ns)
        self.writer.write(CAM_3_TOPIC_NAME, serialize_message(color_right_img_msg), timestamp_ns)


    def publish_tf_static(self) -> None:
        """
        Publish static TF transforms between coordinate frames.

        Creates and publishes time-invariant transforms between base_link, 
        odom, velo_link, and camera frames. Includes a 90-degree rotation
        on the X-axis for visualization compatibility and KITTI-specific
        calibration transforms.

        The static transforms include:
        - map to odom (with 90° X rotation for visualization)
        - base_link to camera_gray_left (identity)
        - base_link to velo_link (from KITTI calibration)
        - base_link to imu_link (if raw data available)

        Parameters
        ----------
        None

        Returns
        -------
        None
        """


        first_timestamp_ns = int(self.kitti_odometry.timestamps[0].total_seconds() * 1e9)
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
            ('base_link', 'velo_link',self.kitti_odometry.calib.T_cam0_velo),  # Invert the transformation for the static TF
            ( 'camera_gray_right','velo_link', self.kitti_odometry.calib.T_cam1_velo),
            ('camera_color_left', 'velo_link', self.kitti_odometry.calib.T_cam2_velo),
            ('camera_color_right', 'velo_link', self.kitti_odometry.calib.T_cam3_velo),
        ]
        if self.kitti_raw is not None:
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
            
        self.writer.write(TF_STATIC_TOPIC_NAME, serialize_message(tfm_static),  current_ros_time.nanoseconds)

    def publish_imu_data(self, topic: str, timestamp_ns: int, counter: int) -> None:
        """
        Publish IMU data from KITTI OXTS measurements.

        Extracts inertial measurement data from KITTI raw dataset's OXTS
        system and converts it to ROS IMU message format. Includes linear
        acceleration, angular velocity, and orientation from OXTS packet.

        Parameters
        ----------
        topic : str
            Name of the ROS topic to publish IMU data to.
        timestamp_ns : int
            Timestamp in nanoseconds for the current frame.
        counter : int
            Frame index to access corresponding OXTS data.

        Returns
        -------
        None
        
        Notes
        -----
        Returns early if counter exceeds available OXTS data length.
        Covariance matrices are set to unknown (-1.0) as KITTI doesn't
        provide uncertainty estimates.
        
        Examples
        --------
        >>> node = KittiOdom2Bag()
        >>> node.publish_imu_data('/kitti/oxts/imu', 1234567890, 42)
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

    def publish_gps_data(self, topic: str, timestamp_ns: int, counter: int) -> None:
        """
        Publish GPS data from KITTI OXTS measurements as NavSatFix.

        Parameters
        ----------
        topic : str
            Name of the ROS topic to publish GPS data to.
        timestamp_ns : int
            Timestamp in nanoseconds for the current frame.
        counter : int
            Frame index to access corresponding OXTS data.
        """
        if counter >= len(self.kitti_raw.oxts):
            return
        oxts = self.kitti_raw.oxts[counter]

        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.frame_id = 'imu_link'
        gps_msg.header.stamp.sec = timestamp_ns // 1_000_000_000
        gps_msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = float(oxts.packet.lat)
        gps_msg.longitude = float(oxts.packet.lon)
        gps_msg.altitude = float(oxts.packet.alt)
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.writer.write(topic, serialize_message(gps_msg), timestamp_ns)

    def publish_velo(self, topic: str, timestamp_ns: int, counter: int) -> None:
        """
        Publish Velodyne LiDAR point cloud to specified topic.

        Reads binary Velodyne point cloud data from KITTI dataset, converts
        it to ROS PointCloud2 message format with XYZI fields, and writes
        to the bag file. Each point contains 3D coordinates and intensity.

        Parameters
        ----------
        topic : str
            Name of the ROS topic to publish point cloud data to.
        timestamp_ns : int
            Timestamp in nanoseconds for the current frame.
        counter : int
            Frame index to access corresponding Velodyne file.

        Returns
        -------
        None
        
        Notes
        -----
        Point cloud data is expected in binary format with 4 floats per point
        (x, y, z, intensity). Sets frame_id to 'velo_link' for proper TF
        transform chain.
        
        Examples
        --------
        >>> node = KittiOdom2Bag()
        >>> node.publish_velo('/kitti/velo/pointcloud', 1234567890, 42)
        """
        velo_filename=self.kitti_odometry.velo_files[counter]
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

    def publish_camera_info(self, camera_id: int, topic: str, timestamp: int) -> None:
        """
        Publish camera calibration information using KITTI projection matrices.

        Creates and publishes CameraInfo messages containing camera calibration
        data derived from KITTI's projection (P_rect_XX) and intrinsic (K_camN)
        matrices for the specified camera.

        Parameters
        ----------
        camera_id : int
            Camera index (0: gray left, 1: gray right, 2: color left, 3: color right).
        topic : str
            ROS topic name for publishing the CameraInfo message.
        timestamp : int
            Timestamp in nanoseconds for the message.

        Returns
        -------
        None
        
        Examples
        --------
        >>> node = KittiOdom2Bag()
        >>> node.publish_camera_info(0, '/kitti/camera_gray_left/camera_info', 1234567890)
        """
        
        cam_id = f"00" if camera_id == 0 else f"10" if camera_id == 1 else f"20" if camera_id == 2 else f"30"
        P= getattr(self.kitti_odometry.calib, f'P_rect_{cam_id}')
        K= getattr(self.kitti_odometry.calib, f'K_cam{camera_id}')
        camera_info_msg_2 = CameraInfo()
        camera_info_msg_2.p = P.flatten() #Projection matrix
        camera_info_msg_2.k = K.flatten() #Intrinsic matrix
        # camera_info_msg_2.k = mtx[:3, :3] #Intrinsic matrix
        self.writer.write(topic, serialize_message(camera_info_msg_2), timestamp)   
        return
        
    def publish_dynamic_tf(self, translation: np.ndarray, quaternion: np.ndarray, timestamp_ns: int) -> None:
        """
        Publish dynamic transform from odom to base_link frame.

        Creates and publishes time-varying TF transform representing the
        vehicle's pose in the odometry frame. This transform updates with
        each frame to reflect the vehicle's motion through the environment.

        Parameters
        ----------
        translation : np.ndarray
            3-element array containing XYZ position coordinates in meters.
        quaternion : np.ndarray
            4-element array with orientation quaternion in [x, y, z, w] format.
        timestamp_ns : int
            Timestamp in nanoseconds for the transform.

        Returns
        -------
        None
        
        Examples
        --------
        >>> import numpy as np
        >>> node = KittiOdom2Bag()
        >>> pos = np.array([1.0, 2.0, 0.0])
        >>> quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity rotation
        >>> node.publish_dynamic_tf(pos, quat, 1234567890)
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
        self.writer.write(TF_TOPIC_NAME, serialize_message(tf_oxts_msg), timestamp_ns)

    def publish_odom(self, translation: np.ndarray, quaternion: np.ndarray, timestamp_ns: int) -> None:
        """
        Publish ground truth odometry and update path trajectory.

        Creates Odometry message from ground truth pose data and publishes
        it to the odometry topic. Also updates the accumulated path message
        by calling publish_path with the current odometry data.

        Parameters
        ----------
        translation : np.ndarray
            3-element array representing XYZ position in meters.
        quaternion : np.ndarray
            4-element array with orientation quaternion in [x, y, z, w] format.
        timestamp_ns : int
            Timestamp in nanoseconds for the odometry message.

        Returns
        -------
        None
        
        Notes
        -----
        Sets child_frame_id to 'camera_gray_left' as the reference frame
        for odometry measurements, with parent frame 'odom'.
        
        Examples
        --------
        >>> import numpy as np
        >>> node = KittiOdom2Bag()
        >>> pos = np.array([1.0, 2.0, 0.0])
        >>> quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity rotation
        >>> node.publish_odom(pos, quat, 1234567890)
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

    def publish_path(self, odom_msg: Odometry, timestamp_ns: int) -> None:
        """
        Add current pose to path trajectory and publish updated path.

        Extracts pose information from an Odometry message, creates a
        PoseStamped message, appends it to the accumulated path, and
        publishes the complete trajectory for visualization.

        Parameters
        ----------
        odom_msg : nav_msgs.msg.Odometry
            Odometry message containing current pose information.
        timestamp_ns : int
            Timestamp in nanoseconds for the path message.

        Returns
        -------
        None
        
        Notes
        -----
        The path accumulates all poses throughout the dataset processing,
        creating a complete trajectory visualization in the 'odom' frame.
        
        Examples
        --------
        >>> from nav_msgs.msg import Odometry
        >>> node = KittiOdom2Bag()
        >>> odom = Odometry()
        >>> # ... populate odom message ...
        >>> node.publish_path(odom, 1234567890)
        """

        #GT pose
        pose = PoseStamped()
        pose.header.frame_id = "odom"  # Use the same frame_id as the Path header
        pose.pose = odom_msg.pose.pose

        self.p_msg.poses.append(pose)
        self.writer.write(PATH_TOPIC_NAME, serialize_message(self.p_msg), timestamp_ns)
        return

def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the KITTI to ROS 2 bag conversion node.
    
    Initializes ROS 2, creates the KittiOdom2Bag node, processes all frames,
    and handles shutdown gracefully with proper cleanup.

    Parameters
    ----------
    args : Optional[List[str]], default=None
        Command line arguments to pass to rclpy.init().

    Returns
    -------
    None
    
    Examples
    --------
    >>> main()  # Run with default arguments
    >>> main(['--ros-args', '--log-level', 'debug'])  # Run with log level
    """
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