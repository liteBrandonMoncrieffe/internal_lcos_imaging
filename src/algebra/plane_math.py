import numpy as np
import logging
from typing import List


def are_points_not_colinear(points: np.ndarray) -> bool:
    """
    Determines whether at least 3 points in the input set are not colinear.

    Parameters:
        points (np.ndarray): A (N x 3) array of points, where each row is a 3D point.
                             Must contain at least 3 points.

    Returns:
        bool: True if at least 3 points are not colinear, False otherwise.
    """
    if not isinstance(points, np.ndarray):
        raise TypeError("Input must be a numpy array.")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Input must be a 2D numpy array with shape (N, 3).")
    if points.shape[0] < 3:
        raise ValueError("At least 3 points are required.")

    reference_point = points[0]
    direction_vector = points[1] - reference_point

    for i in range(2, points.shape[0]):
        test_vector = points[i] - reference_point
        cross = np.cross(direction_vector, test_vector)
        if not np.allclose(cross, 0, atol=1e-8):
            return True  

    return False  

def fit_plane_least_squares(points: np.ndarray,
                            positive_reference_point: np.ndarray,
                            logger: logging.Logger) -> np.ndarray:
    """
    Fits a plane to a set of 3D points using least squares.

    Parameters:
        points (np.ndarray): An (N x 3) array where each row is a 3D point (x, y, z).
                             Requires at least 3 non-colinear points.
        positive_reference_point (np.ndarray): A 3D point (x, y, z) that is expected to be on the positive side of the plane.

    Returns:
        np.ndarray: A 4-element array [a, b, c, d] representing the plane equation:
                    ax + by + cz + d = 0, with the normal vector (a, b, c) normalized.
    """
    if not isinstance(points, np.ndarray):
        logger.error("Plane fit input points must be a numpy array.")
        raise TypeError("Plane fit input points must be a numpy array.")
    print(points)
    if points.ndim != 2 or points.shape[1] != 3:
        logger.error("Plane fit input points must be a 2D numpy array with shape (N, 3).")
        raise ValueError("Plane fit input points must be a 2D numpy array with shape (N, 3).")
    if points.shape[0] < 3:
        logger.error("At least 3 points are required to define a plane.")
        raise ValueError("At least 3 points are required to define a plane.")

    if not are_points_not_colinear(points):
        logger.error("Plane fit input points must be non-colinear.")
        raise ValueError("Plane fit input points must be non-colinear.")

    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    _, _, vh = np.linalg.svd(centered_points)

    normal_vector = vh[-1, :]
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    d = -np.dot(normal_vector, centroid)

    plane_parameters = np.append(normal_vector, d)
    plane_parameters = ensure_correct_normal(plane_parameters, positive_reference_point, logger)

    return plane_parameters

def ensure_correct_normal(plane_parameters: np.ndarray, reference_point: np.ndarray, logger: logging.Logger) -> np.ndarray:
    """
    Ensures the normal vector of a plane points toward the given reference point.

    Parameters:
        plane_parameters (np.ndarray): A 4-element array [a, b, c, d] defining the plane ax + by + cz + d = 0.
        reference_point (np.ndarray): A 3-element array [x, y, z] representing a point expected to be on the positive side.

    Returns:
        np.ndarray: The plane parameters with the normal vector oriented correctly.
                    Returned unchanged if the reference point is already on the positive side.
    """
    if not isinstance(plane_parameters, np.ndarray) or not isinstance(reference_point, np.ndarray):
        raise TypeError("Both plane_parameters and reference_point must be numpy arrays.")

    if plane_parameters.shape != (4,):
        raise ValueError(f"plane_parameters must be a 4-element numpy array got {plane_parameters.shape}.")

    if reference_point.shape != (3,):
        raise ValueError(f"reference_point must be a 3-element numpy array, got {reference_point.shape}.")

    a, b, c, d = plane_parameters
    x, y, z = reference_point

    value = a * x + b * y + c * z + d

    if value >= 0:
        return plane_parameters
    else:
        return -plane_parameters
    
    
def offset_plane(plane_params, offset_distance):
    """
    Offsets a plane along its normal vector by a given distance.

    Parameters:
    plane_params (list or array): Plane parameters [a, b, c, d] for ax + by + cz + d = 0.
    offset_distance (float): Distance to offset the plane along the normal vector.

    Returns:
    list: New plane parameters [a, b, c, d] for the offset plane.
    """
    # Extract normal vector and d
    normal_vector = np.array(plane_params[:3], dtype=np.float64)
    d = plane_params[3]
    
    # Normalize the normal vector if it's not a unit vector
    normal_magnitude = np.linalg.norm(normal_vector)
    if normal_magnitude == 0:
        raise ValueError("The normal vector cannot be zero.")
    
    unit_normal_vector = normal_vector / normal_magnitude
    
    # Offset the d value by the distance along the normal vector
    new_d = d - offset_distance * normal_magnitude
    
    # Return the new plane parameters with the offset
    offset_plane_params = [*unit_normal_vector, new_d]#.tolist()
    return offset_plane_params

def project_point_onto_plane(plane_params: List[float], out_of_plane_point: List[float], logger: logging.Logger):
    point_orientation = None
    point = out_of_plane_point
    if len(out_of_plane_point) == 6:
        point = out_of_plane_point[:3]
        point_orientation = out_of_plane_point[3:6]
    if not len(point) == 3:
        raise ValueError("Point must be a list of 6 elements or a list of 3 elements")
    
    logger.info(f"Projecting point {point} onto plane with parameters {plane_params}\n")
    # Unpack the plane parameters
    normal = np.array(plane_params[:3],dtype=np.float64)
    d = plane_params[3]
    # Compute the distance from the point to the plane
    distance = (np.dot(normal, point) + d) / np.linalg.norm(normal)
    # Compute the projection
    projection = point - distance * normal
    logger.info(f"Projection result: {projection}\n")

    if point_orientation is not None:
        # If the point has an orientation, append it to the projection
        projection = np.concatenate((projection, point_orientation))
        logger.info(f"Projection with orientation: {projection}\n")

    return projection