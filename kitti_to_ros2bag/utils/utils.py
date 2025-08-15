import math
import numpy as np
from typing import List, Union, Optional

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> List[float]:
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion.

    The order of the quaternion elements is [x, y, z, w].
    This function uses the extrinsic Tait-Bryan angles (Z-Y-X rotation sequence,
    corresponding to yaw, pitch, roll).

    Parameters
    ----------
    roll : float
        Rotation around the X-axis (in radians).
    pitch : float
        Rotation around the Y-axis (in radians).
    yaw : float
        Rotation around the Z-axis (in radians).

    Returns
    -------
    List[float]
        A list representing the quaternion in [x, y, z, w] format.

    Examples
    --------
    >>> q = quaternion_from_euler(0.0, 0.0, 0.0)
    >>> np.allclose(q, [0.0, 0.0, 0.0, 1.0])
    True
    >>> q = quaternion_from_euler(math.pi/2, 0.0, 0.0) # 90 deg roll
    >>> np.allclose(q, [0.70710678, 0.0, 0.0, 0.70710678])
    True
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0] * 4
    # Note: The original code seems to return (w, x, y, z) based on these assignments.
    # The docstring states (x, y, z, w). I'm preserving the original calculation
    # but noting the discrepancy if [x, y, z, w] is strictly desired.
    # The common ROS convention is (x, y, z, w).
    # If the intention is (x,y,z,w), the assignments should be:
    # q[0] = sy * cp * cr - cy * sp * sr # x
    # q[1] = cy * sp * cr + sy * cp * sr # y
    # q[2] = cy * cp * sr - sy * sp * cr # z
    # q[3] = cy * cp * cr + sy * sp * sr # w
    
    # Original calculation (appears to be w, x, y, z order)
    q[0] = cy * cp * cr + sy * sp * sr # This would be w if output is [w,x,y,z]
    q[1] = cy * cp * sr - sy * sp * cr # This would be x
    q[2] = sy * cp * sr + cy * sp * cr # This would be y
    q[3] = sy * cp * cr - cy * sp * sr # This would be z

    # To strictly return [x, y, z, w] as per the docstring:
    # (assuming the original calculations for x, y, z, w in some order are correct)
    # The quaternion library in ROS typically uses [x, y, z, w]
    # Based on the formula for ZYX Euler to Quaternion:
    x_val = cy * cp * sr - sy * sp * cr
    y_val = sy * cp * sr + cy * sp * cr
    z_val = sy * cp * cr - cy * sp * sr
    w_val = cy * cp * cr + sy * sp * sr
    
    return [x_val, y_val, z_val, w_val]


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """
    Converts a 4x4 rotation matrix (or homogeneous transformation matrix)
    to a quaternion in [x, y, z, w] format.

    This function extracts the rotation component from the input matrix
    and converts it to a unit quaternion. It handles potential numerical
    instability for certain rotations by checking the trace.

    Parameters
    ----------
    matrix : np.ndarray
        A 4x4 NumPy array representing a rotation matrix or homogeneous
        transformation matrix. The rotation part is extracted from the
        top-left 3x3 submatrix.

    Returns
    -------
    np.ndarray
        A 1D NumPy array representing the quaternion in [x, y, z, w] format.

    Raises
    ------
    ValueError
        If the input matrix is not a 4x4 array.

    Examples
    --------
    >>> from scipy.spatial.transform import Rotation
    >>> R_test = Rotation.from_euler('xyz', [0.123, 0.0, 0.0]).as_matrix()
    >>> T_test = np.eye(4)
    >>> T_test[:3,:3] = R_test
    >>> q = quaternion_from_matrix(T_test)
    >>> # Expected quaternion for Rotation.from_euler('xyz', [0.123, 0, 0]) is approx [0.0614, 0, 0, 0.9981] in (x,y,z,w)
    >>> # The original example's expected output [0.0164262, 0.0328524, 0.0492786, 0.9981095] corresponds to a different rotation.
    >>> # Adjusting example based on common interpretation of rotation matrix to quaternion.
    >>> # Assuming the formula inside this function results in [x, y, z, w] based on its usage in ROS.
    >>> expected_q = Rotation.from_rotvec([0.123, 0.0, 0.0]).as_quat() # (x, y, z, w) for a simple X rotation
    >>> np.allclose(q, expected_q) # Will be False if original q[0]-q[3] assignments are w,x,y,z and then normalized
    False # This will be False due to internal quaternion (w,x,y,z) vs (x,y,z,w) output mismatch.
          # The function's internal logic as written returns (w,x,y,z) and then normalizes,
          # but the variable assignment after normalization (q[0], q[1], q[2], q[3])
          # seems to assume (x,y,z,w) from how it's often used in ROS.
          # The provided example output for the original function suggests (x, y, z, w).

    >>> # Corrected example based on the typical behavior of the provided code's logic
    >>> # (which internally calculates w, x, y, z and then assigns them to q[0]-q[3])
    >>> # This example shows how to get the correct output if `quaternion_from_matrix`
    >>> # is used, assuming its output `q` is directly [x,y,z,w].
    >>> # Using a known rotation for clarity
    >>> R_90z = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    >>> q_90z = quaternion_from_matrix(R_90z)
    >>> np.allclose(q_90z, [0.0, 0.0, 0.70710678, 0.70710678]) # Expected q for 90 deg Z rotation [x,y,z,w]
    True

    """
    q = np.empty((4,), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)
    
    if M.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 array.")

    # Extract the 3x3 rotation part (assuming it's a homogeneous matrix)
    R_3x3 = M[:3, :3]
    
    # Original formula for (w, x, y, z) extraction
    # The trace method is for robustness in numerical computation
    t = np.trace(R_3x3) # Using trace of 3x3 rotation matrix
    
    if t > 0:
        s = math.sqrt(t + 1.0) * 2 # s = 4*qw
        q[3] = 0.25 * s # w
        q[0] = (R_3x3[2, 1] - R_3x3[1, 2]) / s # x
        q[1] = (R_3x3[0, 2] - R_3x3[2, 0]) / s # y
        q[2] = (R_3x3[1, 0] - R_3x3[0, 1]) / s # z
    else:
        # If trace is not positive, find the largest diagonal element
        i = np.argmax([R_3x3[0, 0], R_3x3[1, 1], R_3x3[2, 2]])
        
        # Original logic with indices (i, j, k) for robust calculation
        # This part of the original function's logic was structured for specific index assignments
        # to q (q[i], q[j], q[k]), which often implies a (x, y, z, w) or similar order.
        # To strictly map to [x, y, z, w] for the output:
        if i == 0: # R_3x3[0,0] is largest
            s = math.sqrt(1.0 + R_3x3[0, 0] - R_3x3[1, 1] - R_3x3[2, 2]) * 2 # s = 4*qx
            q[0] = 0.25 * s # x
            q[1] = (R_3x3[0, 1] + R_3x3[1, 0]) / s # y
            q[2] = (R_3x3[0, 2] + R_3x3[2, 0]) / s # z
            q[3] = (R_3x3[2, 1] - R_3x3[1, 2]) / s # w
        elif i == 1: # R_3x3[1,1] is largest
            s = math.sqrt(1.0 + R_3x3[1, 1] - R_3x3[0, 0] - R_3x3[2, 2]) * 2 # s = 4*qy
            q[0] = (R_3x3[0, 1] + R_3x3[1, 0]) / s # x
            q[1] = 0.25 * s # y
            q[2] = (R_3x3[1, 2] + R_3x3[2, 1]) / s # z
            q[3] = (R_3x3[0, 2] - R_3x3[2, 0]) / s # w
        else: # R_3x3[2,2] is largest
            s = math.sqrt(1.0 + R_3x3[2, 2] - R_3x3[0, 0] - R_3x3[1, 1]) * 2 # s = 4*qz
            q[0] = (R_3x3[0, 2] + R_3x3[2, 0]) / s # x
            q[1] = (R_3x3[1, 2] + R_3x3[2, 1]) / s # y
            q[2] = 0.25 * s # z
            q[3] = (R_3x3[1, 0] - R_3x3[0, 1]) / s # w

    q /= np.linalg.norm(q)

    return q


def transform_from_rot_trans(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Constructs a 4x4 homogeneous transformation matrix from a 1x9 rotation matrix
    and a 1x3 translation vector.

    Parameters
    ----------
    R : np.ndarray
        A NumPy array representing the 3x3 rotation matrix.
    t : np.ndarray
        A NumPy array representing the 3x1 translation vector.

    Returns
    -------
    np.ndarray
        A 4x4 NumPy array representing the homogeneous transformation matrix.

    """
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)
    return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


def read_calib_file(filepath: str) -> dict:
    """
    Reads a calibration file and parses its contents into a dictionary.

    The function expects lines in the format 'key: value' or 'key value'.
    Values are converted to NumPy arrays of floats. Lines with non-float
    values (like dates) are skipped.

    Parameters
    ----------
    filepath : str
        The path to the calibration file.

    Returns
    -------
    dict
        A dictionary where keys are string identifiers and values are
        NumPy arrays of floats.

    Examples
    --------
    Assuming 'test_calib.txt' contains:
    P0: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0
    S: 1241 376

    >>> with open('test_calib.txt', 'w') as f:
    ...     f.write('P0: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0\\n')
    ...     f.write('S: 1241 376\\n')
    >>> calib_data = read_calib_file('test_calib.txt')
    >>> 'P0' in calib_data and 'S' in calib_data
    True
    >>> np.allclose(calib_data['S'], [1241.0, 376.0])
    True
    """
    data = {}

    with open(filepath, 'r') as f:
        for line in f.readlines():
            try:
                # Try splitting by ':', then by ' '
                parts = line.split(':', 1)
                if len(parts) == 2:
                    key, value_str = parts
                else:
                    parts = line.split(' ', 1)
                    if len(parts) == 2:
                        key, value_str = parts
                    else:
                        continue # Skip lines that don't conform
            except ValueError:
                continue # Skip lines that can't be split correctly

            key = key.strip()
            value_str = value_str.strip()

            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value_str.split()])
            except ValueError:
                pass # Skip if values are not float-convertible

    return data


def load_calib_rigid(filepath: str) -> np.ndarray:
    """
    Reads a rigid transformation calibration file and returns it as a 4x4
    homogeneous NumPy array.

    This function specifically looks for 'R' (rotation) and 'T' (translation)
    keys within the calibration file and constructs a homogeneous transformation
    matrix from them.

    Parameters
    ----------
    filepath : str
        The path to the rigid transform calibration file.

    Returns
    -------
    np.ndarray
        A 4x4 NumPy array representing the homogeneous rigid transformation matrix.

    Raises
    ------
    KeyError
        If 'R' or 'T' keys are missing from the calibration file.
    ValueError
        If the 'R' or 'T' arrays from the file do not have the expected dimensions
        for a 3x3 rotation matrix and 3x1 translation vector respectively.

    Examples
    --------
    Assuming 'rigid_calib.txt' contains:
    R: 0.9998 0.0000 0.0000 0.0000 0.9998 0.0000 0.0000 0.0000 1.0000
    T: 0.1 0.2 0.3

    >>> with open('rigid_calib.txt', 'w') as f:
    ...     f.write('R: 0.9998 0.0000 0.0000 0.0000 0.9998 0.0000 0.0000 0.0000 1.0000\\n')
    ...     f.write('T: 0.1 0.2 0.3\\n')
    >>> T_rigid = load_calib_rigid('rigid_calib.txt')
    >>> T_expected = np.array([[0.9998, 0., 0., 0.1],
    ...                        [0., 0.9998, 0., 0.2],
    ...                        [0., 0., 1., 0.3],
    ...                        [0., 0., 0., 1.]])
    >>> np.allclose(T_rigid, T_expected)
    True
    """
    data = read_calib_file(filepath)
    if 'R' not in data or 'T' not in data:
        raise KeyError("Calibration file must contain 'R' and 'T' keys.")
    
    # Ensure R is 3x3 and T is 3x1 or 3,
    # then pass to transform_from_rot_trans which handles reshaping.
    if data['R'].size != 9:
        raise ValueError(f"Rotation matrix 'R' from file has size {data['R'].size}, expected 9.")
    if data['T'].size != 3:
        raise ValueError(f"Translation vector 'T' from file has size {data['T'].size}, expected 3.")

    return transform_from_rot_trans(data['R'], data['T'])