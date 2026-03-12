import numpy as np
import logging
from logging_setup.log_config import setup_logging


def normalize_vector(vector):
    """
    Normalize a vector to have unit length.
    If the vector length is very small, return a default axis [1, 0, 0].
    """
    vector = np.asarray(vector, dtype=float)
    vector_norm = np.linalg.norm(vector)

    if vector_norm < 1e-12:
        return np.array([1.0, 0.0, 0.0], dtype=float)

    return vector / vector_norm


def clamp_scalar_value(scalar_value, minimum_value=-1.0, maximum_value=1.0):
    """
    Clamp a scalar floating-point value to the range [minimum_value, maximum_value].
    This is useful before calling functions like arccos.
    """
    return max(minimum_value, min(maximum_value, scalar_value))


def convert_axis_angle_to_unit_quaternion(axis_angle, angle_tolerance=1e-8):
    """
    Convert axis-angle vector to a unit quaternion [w, x, y, z].

    The axis-angle vector is defined as:
        axis_angle = rotation_axis * rotation_angle

    where:
        - rotation_axis is a unit 3D vector
        - rotation_angle is in radians

    Parameters
    ----------
    axis_angle : np.ndarray
        A 3-element array representing axis * angle (radians).
    angle_tolerance : float, optional
        Threshold below which we treat the rotation as "zero" rotation.

    Returns
    -------
    unit_quaternion : np.ndarray
        A 4-element array [w, x, y, z] representing a unit quaternion.
    """
    axis_angle = np.asarray(axis_angle, dtype=float)
    if axis_angle.shape != (3,):
        raise ValueError("axis_angle must be a 3-element array")

    rotation_angle = np.linalg.norm(axis_angle)

    # If the angle is very small, return identity rotation
    if rotation_angle < angle_tolerance:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    # Extract axis as normalized direction
    rotation_axis = axis_angle / rotation_angle

    half_angle = 0.5 * rotation_angle
    sin_half_angle = np.sin(half_angle)
    cos_half_angle = np.cos(half_angle)

    w = cos_half_angle
    x, y, z = rotation_axis * sin_half_angle

    unit_quaternion = np.array([w, x, y, z], dtype=float)

    # Normalize to be safe against numerical noise
    norm_q = np.linalg.norm(unit_quaternion)
    if norm_q < 1e-12:
        # Fallback to identity
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    return unit_quaternion / norm_q

def convert_unit_quaternion_to_rotation_matrix(unit_quaternion):
    """
    Convert a unit quaternion [w, x, y, z] to a 3x3 rotation matrix.

    Parameters
    ----------
    unit_quaternion : np.ndarray
        A 4-element array [w, x, y, z] representing a (unit) quaternion.

    Returns
    -------
    rotation_matrix : np.ndarray
        A 3x3 rotation matrix.
    """
    unit_quaternion = np.asarray(unit_quaternion, dtype=float)
    if unit_quaternion.shape != (4,):
        raise ValueError("unit_quaternion must be a 4-element array [w, x, y, z]")

    # Ensure it is normalized
    norm_q = np.linalg.norm(unit_quaternion)
    if norm_q < 1e-12:
        # Degenerate quaternion: treat as identity rotation
        w, x, y, z = 1.0, 0.0, 0.0, 0.0
    else:
        unit_quaternion = unit_quaternion / norm_q
        w, x, y, z = unit_quaternion

    # Precompute products
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z

    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    rotation_matrix = np.array([
        [1.0 - 2.0 * (yy + zz),  2.0 * (xy - wz),        2.0 * (xz + wy)],
        [2.0 * (xy + wz),        1.0 - 2.0 * (xx + zz),  2.0 * (yz - wx)],
        [2.0 * (xz - wy),        2.0 * (yz + wx),        1.0 - 2.0 * (xx + yy)]
    ], dtype=float)

    return rotation_matrix

def convert_axis_angle_to_rotation_matrix_via_quaternion(axis_angle, angle_tolerance=1e-8):
    """
    Convert an axis-angle vector to a 3x3 rotation matrix using quaternions.

    This is the inverse of `convert_rotation_matrix_to_axis_angle_via_quaternion`,
    assuming that axis_angle = rotation_axis * rotation_angle.

    Parameters
    ----------
    axis_angle : np.ndarray
        A 3-element array representing axis * angle (radians).
    angle_tolerance : float, optional
        Threshold below which we treat as an identity rotation.

    Returns
    -------
    rotation_matrix : np.ndarray
        A 3x3 rotation matrix.
    """
    unit_quaternion = convert_axis_angle_to_unit_quaternion(
        axis_angle,
        angle_tolerance=angle_tolerance
    )
    rotation_matrix = convert_unit_quaternion_to_rotation_matrix(unit_quaternion)
    return rotation_matrix

def convert_rotation_matrix_to_unit_quaternion(rotation_matrix):
    """
    Convert a 3x3 rotation matrix to a unit quaternion [w, x, y, z].

    Parameters
    ----------
    rotation_matrix : np.ndarray
        A 3x3 rotation matrix.

    Returns
    -------
    unit_quaternion : np.ndarray
        A 4-element array [w, x, y, z] representing a unit quaternion.
    """
    rotation_matrix = np.asarray(rotation_matrix, dtype=float)
    if rotation_matrix.shape != (3, 3):
        raise ValueError("rotation_matrix must be a 3x3 array")

    # Optional: enforce orthonormality via a small correction if you do not trust input
    # Here we assume it's already a reasonably valid rotation matrix.

    matrix_trace = np.trace(rotation_matrix)

    if matrix_trace > 0.0:
        # Standard "trace positive" case
        trace_plus_one = matrix_trace + 1.0
        quaternion_scalar_part = np.sqrt(trace_plus_one) / 2.0
        reciprocal_factor = 0.25 / quaternion_scalar_part

        quaternion_x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * reciprocal_factor
        quaternion_y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * reciprocal_factor
        quaternion_z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * reciprocal_factor

        quaternion = np.array(
            [quaternion_scalar_part, quaternion_x, quaternion_y, quaternion_z],
            dtype=float
        )

    else:
        # Trace is not positive, find the largest diagonal element and compute accordingly
        diagonal_elements = np.diagonal(rotation_matrix)
        index_of_largest_diagonal = np.argmax(diagonal_elements)

        if index_of_largest_diagonal == 0:
            # R[0,0] is largest diagonal element
            s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
            quaternion_scalar_part = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            quaternion_x = 0.25 * s
            quaternion_y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            quaternion_z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s

        elif index_of_largest_diagonal == 1:
            # R[1,1] is largest diagonal element
            s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
            quaternion_scalar_part = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            quaternion_x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            quaternion_y = 0.25 * s
            quaternion_z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s

        else:
            # R[2,2] is largest diagonal element
            s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
            quaternion_scalar_part = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            quaternion_x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            quaternion_y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            quaternion_z = 0.25 * s

        quaternion = np.array(
            [quaternion_scalar_part, quaternion_x, quaternion_y, quaternion_z],
            dtype=float
        )

    # Normalize quaternion to be safe against numerical noise
    quaternion_norm = np.linalg.norm(quaternion)
    if quaternion_norm < 1e-12:
        # Fallback to identity rotation quaternion
        quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        quaternion = quaternion / quaternion_norm

    return quaternion


def convert_unit_quaternion_to_axis_angle(unit_quaternion, angle_tolerance=1e-8):
    """
    Convert a unit quaternion [w, x, y, z] to axis-angle representation.

    Parameters
    ----------
    unit_quaternion : np.ndarray
        A 4-element array [w, x, y, z] representing a unit quaternion.
    angle_tolerance : float, optional
        Threshold below which we treat the rotation as "zero" rotation.

    Returns
    -------
    rotation_axis : np.ndarray
        A 3-element unit vector representing the axis of rotation.
    rotation_angle : float
        The rotation angle in radians, in the range [0, pi].
    """
    unit_quaternion = np.asarray(unit_quaternion, dtype=float)
    if unit_quaternion.shape != (4,):
        raise ValueError("unit_quaternion must be a 4-element array [w, x, y, z]")

    quaternion_scalar_part = unit_quaternion[0]
    quaternion_vector_part = unit_quaternion[1:4]

    # Ensure quaternion is normalized
    quaternion_norm = np.linalg.norm(unit_quaternion)
    if quaternion_norm < 1e-12:
        # Degenerate quaternion, treat as identity rotation
        quaternion_scalar_part = 1.0
        quaternion_vector_part = np.array([0.0, 0.0, 0.0], dtype=float)
    else:
        unit_quaternion = unit_quaternion / quaternion_norm
        quaternion_scalar_part = unit_quaternion[0]
        quaternion_vector_part = unit_quaternion[1:4]

    # Clamp scalar part in case of tiny numerical drift
    quaternion_scalar_part = clamp_scalar_value(quaternion_scalar_part, -1.0, 1.0)

    # angle = 2 * arccos(w)
    rotation_angle = 2.0 * np.arccos(quaternion_scalar_part)

    # If angle is tiny, the axis direction is not well-defined
    if abs(rotation_angle) < angle_tolerance:
        rotation_axis = np.array([1.0, 0.0, 0.0], dtype=float)
        rotation_angle = 0.0
        return rotation_axis, rotation_angle

    # Otherwise, axis = v / sin(theta / 2)
    sine_half_angle = np.sqrt(max(0.0, 1.0 - quaternion_scalar_part * quaternion_scalar_part))

    if sine_half_angle < 1e-12:
        # Numerically, this should correspond to a 180-degree rotation.
        # In that case, quaternion_vector_part already points along the axis.
        rotation_axis = normalize_vector(quaternion_vector_part)
        rotation_angle = np.pi
        return rotation_axis, rotation_angle

    rotation_axis = quaternion_vector_part / sine_half_angle
    rotation_axis = normalize_vector(rotation_axis)

    # Ensure the angle is in [0, pi] and axis is consistently oriented (optional convention)
    if rotation_angle > np.pi + 1e-12:
        # Flip to keep angle <= pi
        rotation_angle = 2.0 * np.pi - rotation_angle
        rotation_axis = -rotation_axis

    return rotation_axis, rotation_angle


def convert_rotation_matrix_to_axis_angle_via_quaternion(rotation_matrix, angle_tolerance=1e-8):
    """
    Convert a 3x3 rotation matrix to axis-angle representation using quaternions.

    This approach is generally more numerically robust than directly using the
    rotation matrix trace and sin(theta) formulas.

    Parameters
    ----------
    rotation_matrix : np.ndarray
        A 3x3 rotation matrix.
    angle_tolerance : float, optional
        Small threshold used to classify near-zero rotations.

    Returns
    -------
    rotation_axis : np.ndarray
        A 3-element unit vector representing the axis of rotation.
    rotation_angle : float
        The rotation angle in radians, in the range [0, pi].
    """
    rotation_matrix = np.asarray(rotation_matrix, dtype=float)
    if rotation_matrix.shape != (3, 3):
        raise ValueError("rotation_matrix must be a 3x3 array")

    # Optionally check determinant to ensure we indeed have a rotation matrix
    determinant_of_rotation_matrix = np.linalg.det(rotation_matrix)
    if not np.isclose(determinant_of_rotation_matrix, 1.0, atol=1e-3):
        raise ValueError(
            f"Input matrix determinant is {determinant_of_rotation_matrix:.6f}, "
            "expected approximately 1.0 for a rotation matrix."
        )

    unit_quaternion = convert_rotation_matrix_to_unit_quaternion(rotation_matrix)
    rotation_axis, rotation_angle = convert_unit_quaternion_to_axis_angle(
        unit_quaternion,
        angle_tolerance=angle_tolerance,
    )

    axis_angle = rotation_axis * rotation_angle
    return axis_angle

def poseToTransformationMatrix(pose: np.ndarray) -> np.ndarray:
    """
    Convert a 6-DOF pose (x, y, z, rx, ry, rz) to a 4x4 transformation matrix.

    Parameters
    ----------
    pose : np.ndarray
        A 6-element array representing the pose:
        [x, y, z, rx, ry, rz], where (rx, ry, rz) is the axis-angle representation."""
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,):
        raise ValueError("pose must be a 6-element array [x, y, z, rx, ry, rz]")

    translation = pose[0:3]
    axis_angle = pose[3:6]

    rotation_matrix = convert_axis_angle_to_rotation_matrix_via_quaternion(axis_angle)

    homogenizedTranslationMatrix = np.eye(4, dtype=float)
    homogenizedTranslationMatrix[0:3, 3] = translation
    homogenizedRotationMatrix = np.eye(4, dtype=float)
    homogenizedRotationMatrix[0:3, 0:3] = rotation_matrix
    transformationMatrix = homogenizedTranslationMatrix @ homogenizedRotationMatrix

    return transformationMatrix


def transformationMatrixToPose(transformation_matrix: np.ndarray) -> np.ndarray:
    """
    Convert a 4x4 transformation matrix to a 6-DOF pose (x, y, z, rx, ry, rz).

    Parameters
    ----------
    transformation_matrix : np.ndarray
        A 4x4 transformation matrix.

    Returns
    -------
    pose : np.ndarray
        A 6-element array representing the pose:
        [x, y, z, rx, ry, rz], where (rx, ry, rz) is the axis-angle representation."""
    transformation_matrix = np.asarray(transformation_matrix, dtype=float)
    if transformation_matrix.shape != (4, 4):
        raise ValueError("transformation_matrix must be a 4x4 array")
    rotation_matrix = transformation_matrix[0:3, 0:3]
    translation = transformation_matrix[0:3, 3]
    axis_angle = convert_rotation_matrix_to_axis_angle_via_quaternion(rotation_matrix)
    pose = np.concatenate((translation, axis_angle))
    return pose



if __name__ == "__main__":

    logger = setup_logging(logging.DEBUG,logging.DEBUG,"test_logs/urmath/","coord_system_math_test")

    #def basis_to_axis_angle(basis_vectors: Union[List[List[float]], np.ndarray], logger) -> np.ndarray:
    origin = np.array([-0.03723, -0.26799, 0.03349])
    y_axis_point = np.array([-0.03675, -0.29147, 0.03349])
    y_axis = (y_axis_point - origin)/np.linalg.norm(y_axis_point - origin)
    z_axis = np.array([0,0,1])
    x_axis = (np.cross(y_axis, z_axis)/np.linalg.norm(np.cross(y_axis, z_axis)))
    z_axis = (np.cross(x_axis, y_axis)/np.linalg.norm(np.cross(x_axis, y_axis)))
    basis = [x_axis, y_axis, z_axis]
    rotation_matrix = np.column_stack(basis)

    axis_angle = convert_rotation_matrix_to_axis_angle_via_quaternion(rotation_matrix)
    logger.info(f"Axis Angle from basis:\n x: {axis_angle[0]}\n y: {axis_angle[1]}\n z: {axis_angle[2]}\n")