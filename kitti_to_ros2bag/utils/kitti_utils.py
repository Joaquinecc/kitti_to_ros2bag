import time
import os
import numpy as np
from pathlib import Path
import cv2
from kitti_to_ros2bag.utils import utils
from collections import namedtuple
import argparse

LEFT_IMG_FOLDER = "image_0"
RIGHT_IMG_FOLDER = "image_1"
VELODYNE_FOLDER = "velodyne"
# DISTANCE = 0.54meters

class KITTIOdometryDataset():
    def __init__(self, data_dir, sequence: int, odom_dir = None,  *_, **__) -> None:
        self.kitti_sequence_dir = os.path.join(data_dir, "sequences", f'{sequence:02d}')
        if odom_dir is not None:
            self.odom_dir = os.path.join(odom_dir, 'poses', f'{sequence:02d}.txt')
        self.left_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, LEFT_IMG_FOLDER)
        self.right_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, RIGHT_IMG_FOLDER)
        self.velodyne_sequence_dir = os.path.join(self.kitti_sequence_dir, VELODYNE_FOLDER)
        self.calib_file = os.path.join(self.kitti_sequence_dir,"calib.txt")
        self.time_file = os.path.join(self.kitti_sequence_dir,"times.txt")
        self._load_calib()

    def write_text(self, files_list, file_name):
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
    
    def left_images(self):
        image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")
        image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def right_images(self):
        image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")
        image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if filename.lower().endswith(image_extensions):
                image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def velodyne_plc(self):
        velodyne_extensions = (".bin",)
        plc_files = []
        for filename in os.listdir(self.velodyne_sequence_dir):
            if filename.lower().endswith(velodyne_extensions):
                plc_files.append(os.path.join(self.velodyne_sequence_dir, filename))
        plc_files = sorted(plc_files)
        return plc_files
    
    def stereo_images(self):
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
    
    def continuous_image_reader(self, images):
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
                
    def projection_matrix(self, cam):
        projection_matrices = []
        with open(self.calib_file, 'r') as file:
            for line in file:
                if line.startswith('P'):
                    values = [float(x) for x in line.split(':')[1].strip().split()]
                    matrix = np.array(values).reshape(3, 4)
                    projection_matrices.append(matrix)
        return projection_matrices[cam]
    
    def times_file(self):
        matrix = []
        with open(self.time_file, 'r') as file:
            for line in file:
                matrix.append(float(line))
        return np.array(matrix)
    
    def odom_pose(self):
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
    
    def __getitem__(self, idx):

        return

    def __len__(self):
        
        return
    def _load_calib(self):
        """Load and compute intrinsic and extrinsic calibration parameters."""
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

def main():
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
    
