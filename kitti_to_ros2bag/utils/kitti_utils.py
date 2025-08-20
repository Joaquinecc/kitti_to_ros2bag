import time
import os
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple, Any
import cv2
from kitti_to_ros2bag.utils import utils
from collections import namedtuple
import argparse

LEFT_IMG_FOLDER = "image_0"
RIGHT_IMG_FOLDER = "image_1"
VELODYNE_FOLDER = "velodyne"
# DISTANCE = 0.54meters

class KITTIOdometryDataset():
    """
    Handler for loading and accessing KITTI Odometry dataset files.
    
    This class provides methods to access stereo images, Velodyne point clouds,
    calibration data, timestamps, and ground truth odometry poses from the
    KITTI Odometry dataset. It handles file path management and calibration
    parameter computation.
    
    Attributes
    ----------
    kitti_sequence_dir : str
        Path to the specific sequence directory in KITTI dataset.
    odom_dir : Optional[str]
        Path to the odometry ground truth file.
    left_cam_sequence_dir : str
        Path to left camera image directory.
    right_cam_sequence_dir : str
        Path to right camera image directory.
    velodyne_sequence_dir : str
        Path to Velodyne point cloud directory.
    calib_file : str
        Path to calibration file for the sequence.
    time_file : str
        Path to timestamps file for the sequence.
    calib : namedtuple
        Calibration parameters including projection matrices and transforms.
    """
    
    def __init__(self, data_dir: str, sequence: int, odom_dir: Optional[str] = None, *_, **__) -> None:
        """
        Initialize KITTI Odometry dataset loader.
        
        Sets up file paths for accessing KITTI dataset components including
        images, point clouds, calibration data, and optionally ground truth
        odometry poses. Loads and computes calibration parameters.

        Parameters
        ----------
        data_dir : str
            Root directory path to the KITTI dataset.
        sequence : int
            Sequence number to load (0-21 for KITTI Odometry).
        odom_dir : Optional[str], default=None
            Directory containing ground truth pose files. If None,
            odometry data will not be available.

        Returns
        -------
        None
        
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0, '/path/to/poses')
        >>> len(dataset.left_images())  # Number of left camera images
        4541
        """
        self.kitti_sequence_dir = os.path.join(data_dir, "sequences", f'{sequence:02d}')
        if odom_dir is not None:
            self.odom_dir = os.path.join(odom_dir, 'poses', f'{sequence:02d}.txt')
        self.left_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, LEFT_IMG_FOLDER)
        self.right_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, RIGHT_IMG_FOLDER)
        self.velodyne_sequence_dir = os.path.join(self.kitti_sequence_dir, VELODYNE_FOLDER)
        self.calib_file = os.path.join(self.kitti_sequence_dir,"calib.txt")
        self.time_file = os.path.join(self.kitti_sequence_dir,"times.txt")
        self._load_calib()

    def write_text(self, files_list: List[str], file_name: str) -> None:
        """
        Write a list of file paths to a text file.
        
        Creates a text file containing one file path per line from the
        provided list. Used for debugging and file listing purposes.

        Parameters
        ----------
        files_list : List[str]
            List of file paths or strings to write to file.
        file_name : str
            Base name for the output file (will be prefixed with 'file_').

        Returns
        -------
        None
        
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> files = ['/path/img1.png', '/path/img2.png']
        >>> dataset.write_text(files, 'image_list')  # Creates 'file_image_list.txt'
        """
        file_name = f"file_{file_name}.txt"
        file_path = os.path.join(self.sequence_dir, file_name)
        try:
            with open(file_path, 'w') as file:
                for item in files_list:
                    file.write(f"{item}\n")
            print(f"File '{file_name}' has been created and written to '{file_path}'.")
        except Exception as e:
            print(f"An error occurred: {e}")
        return
    
    def left_images(self) -> List[str]:
        """
        Get sorted list of left camera image file paths.
        
        Scans the left camera directory for image files with common extensions
        and returns them sorted by filename for sequential processing.

        Parameters
        ----------
        None

        Returns
        -------
        List[str]
            Sorted list of absolute file paths to left camera images.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> left_imgs = dataset.left_images()
        >>> len(left_imgs)  # Number of left images
        4541
        >>> left_imgs[0]  # First image path
        '/path/to/kitti/sequences/00/image_0/000000.png'
        """
        image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")
        image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def right_images(self) -> List[str]:
        """
        Get sorted list of right camera image file paths.
        
        Scans the right camera directory for image files with common extensions
        and returns them sorted by filename for sequential processing.

        Parameters
        ----------
        None

        Returns
        -------
        List[str]
            Sorted list of absolute file paths to right camera images.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> right_imgs = dataset.right_images()
        >>> len(right_imgs)  # Number of right images
        4541
        >>> right_imgs[0]  # First image path
        '/path/to/kitti/sequences/00/image_1/000000.png'
        """
        image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")
        image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if filename.lower().endswith(image_extensions):
                image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def velodyne_plc(self) -> List[str]:
        """
        Get sorted list of Velodyne point cloud file paths.
        
        Scans the Velodyne directory for binary point cloud files (.bin)
        and returns them sorted by filename for sequential processing.

        Parameters
        ----------
        None

        Returns
        -------
        List[str]
            Sorted list of absolute file paths to Velodyne point cloud files.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> velo_files = dataset.velodyne_plc()
        >>> len(velo_files)  # Number of point cloud files
        4541
        >>> velo_files[0]  # First point cloud path
        '/path/to/kitti/sequences/00/velodyne/000000.bin'
        """
        velodyne_extensions = (".bin",)
        plc_files = []
        for filename in os.listdir(self.velodyne_sequence_dir):
            if filename.lower().endswith(velodyne_extensions):
                plc_files.append(os.path.join(self.velodyne_sequence_dir, filename))
        plc_files = sorted(plc_files)
        return plc_files
    
    def stereo_images(self) -> Tuple[List[str], List[str]]:
        """
        Get sorted lists of both left and right camera image file paths.
        
        Combines functionality of left_images() and right_images() methods
        to return both stereo image lists simultaneously.

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[List[str], List[str]]
            A tuple containing (left_image_files, right_image_files) where
            each is a sorted list of absolute file paths.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> left_imgs, right_imgs = dataset.stereo_images()
        >>> len(left_imgs) == len(right_imgs)  # Should have same count
        True
        """
        image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")
        left_image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if filename.lower().endswith(image_extensions):
                left_image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        left_image_files = sorted(left_image_files)

        right_image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if filename.lower().endswith(image_extensions):
                right_image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        right_image_files = sorted(right_image_files)

        return left_image_files, right_image_files
    
    def continuous_image_reader(self, images: str) -> None:
        """
        Display images continuously in OpenCV windows for visualization.
        
        Provides real-time visualization of KITTI images in a loop. Supports
        displaying left, right, or stereo (side-by-side) images with configurable
        display timing.

        Parameters
        ----------
        images : str
            Type of images to display. Options:
            - 'left_images': Display only left camera images
            - 'right_images': Display only right camera images  
            - Any other value: Display stereo images side-by-side

        Returns
        -------
        None
        
        Notes
        -----
        This method runs indefinitely until manually stopped. Images are
        displayed with 55ms delay between frames. Windows are destroyed
        after each stereo display cycle.
        
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> dataset.continuous_image_reader('left_images')  # Display left images
        >>> dataset.continuous_image_reader('stereo')  # Display stereo pairs
        """
        if images == "right_images":
            image_files = self.right_images()
        elif images == "left_images":
            image_files = self.left_images()
        else:
            left_image_files, right_img_files = self.stereo_images()
        while True:
            if images == "right_images" or images == "left_images":
                for image_file in image_files:
                    image = cv2.imread(image_file)
                    window_name = "image display"
                    if image is not None:
                        print(f"Reading image: {image_file}")
                        cv2.imshow(window_name, image)
                        cv2.waitKey(55)
                time.sleep(5)
            else:
                for left_img_file, right_img_file in zip(left_image_files, right_img_files):
                    left_image = cv2.imread(left_img_file)
                    right_image = cv2.imread(right_img_file)
                    window_name = "Stereo display"
                    if left_image is not None and right_image is not None:
                        stereo_image = cv2.hconcat([left_image, right_image])
                        cv2.imshow("Stereo Display", stereo_image)
                        cv2.waitKey(55)
                    else:
                        print("One or both images are invalid.")
                time.sleep(5)
                cv2.destroyAllWindows()
                
    def projection_matrix(self, cam: int) -> np.ndarray:
        """
        Get camera projection matrix for specified camera.
        
        Reads calibration file and extracts the 3x4 projection matrix
        for the specified camera number. KITTI provides 4 projection
        matrices (P0, P1, P2, P3) for different camera configurations.

        Parameters
        ----------
        cam : int
            Camera number (0-3) to get projection matrix for:
            - 0: Left grayscale camera
            - 1: Right grayscale camera  
            - 2: Left color camera
            - 3: Right color camera

        Returns
        -------
        np.ndarray
            3x4 projection matrix for the specified camera.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> P0 = dataset.projection_matrix(0)  # Left grayscale camera
        >>> P0.shape
        (3, 4)
        >>> P1 = dataset.projection_matrix(1)  # Right grayscale camera
        """
        projection_matrices = []
        with open(self.calib_file, 'r') as file:
            for line in file:
                if line.startswith('P'):
                    values = [float(x) for x in line.split(':')[1].strip().split()]
                    matrix = np.array(values).reshape(3, 4)
                    projection_matrices.append(matrix)
        return projection_matrices[cam]
    
    def times_file(self) -> np.ndarray:
        """
        Load timestamp data for all frames in the sequence.
        
        Reads the times.txt file which contains one timestamp per line
        corresponding to each frame in the sequence. Timestamps are
        relative to the start of the sequence.

        Parameters
        ----------
        None

        Returns
        -------
        np.ndarray
            1D array of timestamps in seconds for each frame.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0)
        >>> times = dataset.times_file()
        >>> times.shape  # Number of timestamps
        (4541,)
        >>> times[0]  # First timestamp (usually 0.0)
        0.0
        >>> times[1] - times[0]  # Time difference between frames
        0.1
        """
        matrix = []
        with open(self.time_file, 'r') as file:
            for line in file:
                matrix.append(float(line))
        return np.array(matrix)
    
    def odom_pose(self) -> np.ndarray:
        """
        Load ground truth odometry poses as homogeneous transformation matrices.
        
        Reads the ground truth pose file and converts each 12-element pose
        vector into a 4x4 homogeneous transformation matrix representing
        the camera pose in world coordinates.

        Parameters
        ----------
        None

        Returns
        -------
        np.ndarray
            Array of shape (N, 4, 4) containing homogeneous transformation
            matrices for each frame, where N is the number of poses.
            
        Raises
        ------
        FileNotFoundError
            If the odometry directory/file doesn't exist. Only 10 sequences
            in KITTI have ground truth odometry available.
            
        Examples
        --------
        >>> dataset = KITTIOdometryDataset('/path/to/kitti', 0, '/path/to/poses')
        >>> poses = dataset.odom_pose()
        >>> poses.shape  # (num_frames, 4, 4)
        (4541, 4, 4)
        >>> poses[0]  # First pose (usually identity or close to it)
        array([[1., 0., 0., 0.],
               [0., 1., 0., 0.],
               [0., 0., 1., 0.],
               [0., 0., 0., 1.]])
        """
        if not os.path.exists(self.odom_dir):
            raise FileNotFoundError(f"Odom directory not found: {self.odom_dir}, Ground truth(Odometry) is available for only 10 sequences in KITTI. Stopping the process.")
        with open(self.odom_dir, 'r') as file:
            lines = file.readlines()
        transformation_data = [[float(val) for val in line.split()] for line in lines]
        homogenous_matrix_arr = []
        for i in range(len(transformation_data)):
            homogenous_matrix = np.identity(4)
            homogenous_matrix[0, :] = transformation_data[i][0:4]
            homogenous_matrix[1:2, :] = transformation_data[i][4:8]
            homogenous_matrix[2:3, :] = transformation_data[i][8:12]
            homogenous_matrix_arr.append(homogenous_matrix)
        return np.array(homogenous_matrix_arr)
    
    def __getitem__(self, idx: int) -> Any:
        """
        Get dataset item by index (not implemented).
        
        Placeholder method for dataset indexing functionality.
        Currently not implemented.

        Parameters
        ----------
        idx : int
            Index of the item to retrieve.

        Returns
        -------
        Any
            Dataset item (not implemented).
        """

        return

    def __len__(self) -> int:
        """
        Get length of the dataset (not implemented).
        
        Placeholder method for dataset length functionality.
        Currently not implemented.

        Parameters
        ----------
        None

        Returns
        -------
        int
            Length of the dataset (not implemented).
        """
        
        return
        
    def _load_calib(self) -> None:
        """
        Load and compute intrinsic and extrinsic calibration parameters.
        
        Reads the calibration file for the sequence and computes various
        calibration parameters including projection matrices, camera intrinsics,
        transform matrices between coordinate frames, and stereo baselines.
        
        The computed calibration data includes:
        - P_rect_XX: Rectified projection matrices for cameras 0-3
        - T_camX_velo: Transforms from Velodyne to camera frames
        - K_camX: Camera intrinsic matrices
        - b_gray, b_rgb: Stereo baselines for grayscale and color cameras

        Parameters
        ----------
        None

        Returns
        -------
        None
        
        Notes
        -----
        Results are stored in self.calib as a namedtuple for read-only access.
        This prevents accidental modification of calibration parameters.
        """
        # We'll build the calibration parameters as a dictionary, then
        # convert it to a namedtuple to prevent it from being modified later
        data = {}

        # Load the calibration file
        calib_filepath = os.path.join(self.kitti_sequence_dir, 'calib.txt')
        filedata = utils.read_calib_file(calib_filepath)

        # Create 3x4 projection matrices
        P_rect_00 = np.reshape(filedata['P0'], (3, 4))
        P_rect_10 = np.reshape(filedata['P1'], (3, 4))
        P_rect_20 = np.reshape(filedata['P2'], (3, 4))
        P_rect_30 = np.reshape(filedata['P3'], (3, 4))

        data['P_rect_00'] = P_rect_00
        data['P_rect_10'] = P_rect_10
        data['P_rect_20'] = P_rect_20
        data['P_rect_30'] = P_rect_30

        # Compute the rectified extrinsics from cam0 to camN
        T1 = np.eye(4)
        T1[0, 3] = P_rect_10[0, 3] / P_rect_10[0, 0]
        T2 = np.eye(4)
        T2[0, 3] = P_rect_20[0, 3] / P_rect_20[0, 0]
        T3 = np.eye(4)
        T3[0, 3] = P_rect_30[0, 3] / P_rect_30[0, 0]

        # Compute the velodyne to rectified camera coordinate transforms
        data['T_cam0_velo'] = np.reshape(filedata['Tr'], (3, 4))
        data['T_cam0_velo'] = np.vstack([data['T_cam0_velo'], [0, 0, 0, 1]])
        data['T_cam1_velo'] = T1.dot(data['T_cam0_velo'])
        data['T_cam2_velo'] = T2.dot(data['T_cam0_velo'])
        data['T_cam3_velo'] = T3.dot(data['T_cam0_velo'])

        # Compute the camera intrinsics
        data['K_cam0'] = P_rect_00[0:3, 0:3]
        data['K_cam1'] = P_rect_10[0:3, 0:3]
        data['K_cam2'] = P_rect_20[0:3, 0:3]
        data['K_cam3'] = P_rect_30[0:3, 0:3]

        # Compute the stereo baselines in meters by projecting the origin of
        # each camera frame into the velodyne frame and computing the distances
        # between them
        p_cam = np.array([0, 0, 0, 1])
        p_velo0 = np.linalg.inv(data['T_cam0_velo']).dot(p_cam)
        p_velo1 = np.linalg.inv(data['T_cam1_velo']).dot(p_cam)
        p_velo2 = np.linalg.inv(data['T_cam2_velo']).dot(p_cam)
        p_velo3 = np.linalg.inv(data['T_cam3_velo']).dot(p_cam)

        data['b_gray'] = np.linalg.norm(p_velo1 - p_velo0)  # gray baseline
        data['b_rgb'] = np.linalg.norm(p_velo3 - p_velo2)   # rgb baseline

        self.calib = namedtuple('CalibData', data.keys())(*data.values())

def main() -> None:
    """
    Main function for testing KITTI dataset loading functionality.
    
    Provides command-line interface for loading and testing the KITTI
    odometry dataset. Parses command line arguments for dataset path
    and sequence number, then loads the dataset and displays calibration info.

    Parameters
    ----------
    None

    Returns
    -------
    None
    
    Examples
    --------
    Command line usage:
    $ python kitti_utils.py /path/to/kitti --sequence 0
    """
    parser = argparse.ArgumentParser(description="KITTI Odometry Dataset Loader")
    parser.add_argument("dataset_path", help="Path to the root of the KITTI dataset")
    parser.add_argument("--sequence", type=int, default=0, help="Sequence number (default: 0)")
    args = parser.parse_args()

    dataset_path = args.dataset_path
    sequence = args.sequence

    # Use the same path for data and odom (as requested)
    kitti = KITTIOdometryDataset(data_dir=dataset_path, sequence=sequence, odom_dir=dataset_path)
    print('kitti.calib.T_cam0_velo')
    print(kitti.calib.T_cam0_velo)
    # matrix = kitti.projection_matrix(3)
    # right, left = kitti.stereo_images()
    # kitti.times_file()
    # arr = kitti.odom_pose()


if __name__=="__main__":
    main()
    
