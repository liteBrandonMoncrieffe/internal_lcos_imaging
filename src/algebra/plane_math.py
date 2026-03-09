import numpy as np

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