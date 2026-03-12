import numpy as np
from typing import List, Union, Optional, Sequence
import logging


def is_variable_numpy_or_list(variable: Union[List[float],np.ndarray], expected_length: int, logger, variable_name: str ="variable"):
    """
    Validates that a variable is either a numpy array or a list,
    and checks that it has the specified length.
    
    Args:
        variable: The variable to validate (list or numpy array).
        expected_length (int): The required length of the variable.
        variable_name (str): The name of the variable (used in logging messages).
        
    Returns:
        bool: True if the variable is valid (correct type and length), raises exception otherwise.
    
    Raises:
        ValueError: If the variable is not a list or numpy array or correct length.
    """
    if not isinstance(variable, (np.ndarray, list)):
        logger.critical(f"{variable_name} must be a list or numpy array.")
        raise ValueError(f"{variable_name} must be a list or numpy array.")
    
    if len(variable) != expected_length:
        logger.warning(f"{variable_name} does not have the expected length of {expected_length}.")
        raise ValueError(f"{variable_name} does not have the expected length of {expected_length}.")

    return True

def rotate_vector_about_axis(v2: Sequence[float],
                             v1_axis: Sequence[float],
                             theta: float) -> np.ndarray:
    """
    Rotate vector v2 about axis v1_axis by angle theta (radians)
    using Rodrigues' rotation formula.

    Parameters
    ----------
    v2 : Sequence[float]
        The vector to rotate (size 3).
    v1_axis : Sequence[float]
        The rotation axis (size 3). Does not need to be unit length.
    theta : float
        Rotation angle in radians.

    Returns
    -------
    np.ndarray
        The rotated vector (size 3).
    """
    v = np.asarray(v2, dtype=float).reshape(3,)
    k = np.asarray(v1_axis, dtype=float).reshape(3,)

    # Normalize axis; guard against zero-length axis
    k_norm = np.linalg.norm(k)
    if k_norm == 0:
        raise ValueError("Rotation axis (vector 1) must be non-zero.")
    k = k / k_norm

    # Rodrigues’ rotation formula:
    # v_rot = v*cosθ + (k × v)*sinθ + k*(k·v)*(1 - cosθ)
    c = np.cos(theta)
    s = np.sin(theta)
    v_rot = v * c + np.cross(k, v) * s + k * (np.dot(k, v)) * (1 - c)
    return v_rot

def coordinate_system_to_plane_parameters(coordinate_system: List[float], logger: logging.Logger):
    """
    Converts a coordinate system defined by a 6D pose (position + axis-angle rotation)
    into plane parameters [a, b, c, d] for the plane equation ax +"""
    # split off the position and orientation
    origin = coordinate_system[:3]
    axis_angle = coordinate_system[3:]

    rotation_matrix = axis_angle_to_rotation_matrix(axis_angle, logger)

    z_axis = rotation_matrix[:, 2]  # third column is the Z axis
    a, b, c = z_axis
    d = -np.dot(z_axis, origin)

    return [a, b, c, d]


def axis_angle_to_rotation_matrix(axis_angle: Union[List[float], np.ndarray],
                                  logger: Optional[object] = None) -> np.ndarray:
    """
    Converts an axis-angle rotation vector into a 3x3 rotation matrix
    using Rodrigues' rotation formula.

    Parameters
    ----------
    axis_angle : list[float] or np.ndarray
        3-element rotation vector [rx, ry, rz], where:
        - angle (rad) = ||axis_angle||
        - axis = axis_angle / ||axis_angle||
    logger : optional
        Logger with .debug(...) method for optional debug printing.

    Returns
    -------
    np.ndarray
        3x3 rotation matrix corresponding to the axis-angle input.
    """
    axis_angle = np.asarray(axis_angle, dtype=float).reshape(3,)
    theta = np.linalg.norm(axis_angle)
    eps = 1e-12

    if theta < eps:
        # No rotation → Identity matrix
        if logger: logger.debug("Small rotation angle detected: returning identity matrix.")
        return np.eye(3)

    # Normalize axis
    k = axis_angle / theta
    kx, ky, kz = k

    # Skew-symmetric matrix
    K = np.array([[0, -kz, ky],
                  [kz, 0, -kx],
                  [-ky, kx, 0]], dtype=float)

    s = np.sin(theta)
    c = np.cos(theta)

    # Rodrigues' rotation formula
    R = np.eye(3) + s * K + (1 - c) * (K @ K)

    if logger:
        logger.debug(f"axis (unit): {k}")
        logger.debug(f"angle (rad): {theta}")
        logger.debug(f"rotation matrix:\n{R}")

    return R

def rotation_matrix_to_axis_angle(R):
    """
    Converts a rotation matrix to an axis-angle representation.
    
    Parameters:
        R (np.ndarray): A 3x3 rotation matrix.
    
    Returns:
        axis (np.ndarray): The unit vector representing the axis of rotation.
        angle (float): The rotation angle in radians.
    """
    assert R.shape == (3, 3), "R must be a 3x3 matrix."
    
    # Compute the angle (theta) from the trace of the rotation matrix
    angle = np.arccos((np.trace(R) - 1) / 2)
    
    # Handle the case when the angle is 0 (no rotation)
    if np.isclose(angle, 0):
        return np.array([0, 0, 0])*angle
    
    # Handle the case when the angle is pi (180 degrees)
    if np.isclose(angle, np.pi):
        # Special case where the axis can be any orthogonal vector to the rotation plane
        # We can derive the axis from the eigenvalues of the rotation matrix
        eigenvalues, eigenvectors = np.linalg.eig(R)
        axis = eigenvectors[:, np.isclose(eigenvalues, 1)].flatten().real
        return (axis / np.linalg.norm(axis))*angle
    
    # Compute the axis of rotation
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    
    axis = axis/np.linalg.norm(axis)  # Normalize the axis
    
    return angle*(axis)


def basis_to_axis_angle(basis_vectors: Union[List[List[float]], np.ndarray], logger) -> np.ndarray:
    """
    Converts three orthogonal basis vectors into an axis-angle representation.
    
    Parameters:
        basis_vectors (list of np.ndarray): A list or array of three orthogonal basis vectors.
                                            Each vector is a 3D vector representing one axis (x, y, z).
    
    Returns:
        axis (np.ndarray): The unit vector representing the axis of rotation.
        angle (float): The rotation angle in radians.
    """

    for vector in basis_vectors:
        is_variable_numpy_or_list(vector, 3, logger)

    # Ensure the input is a numpy array
    basis_vectors = np.array(basis_vectors)

    for i in range(3):
        if not np.isclose(np.linalg.norm(basis_vectors[i]), 1.0):
            basis_vectors[i] = basis_vectors[i] / np.linalg.norm(basis_vectors[i])
            logger.warning(f"Basis vector {i} was not unit length. It has been normalized.")
    
    # Construct the rotation matrix from the basis vectors
    # Each vector (v_x, v_y, v_z) is a column of the rotation matrix
    R = np.column_stack(basis_vectors)
    
    # Convert the rotation matrix to axis-angle
    axis_angle = rotation_matrix_to_axis_angle(R)
    return axis_angle
