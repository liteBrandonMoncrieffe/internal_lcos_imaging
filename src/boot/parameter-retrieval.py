import logging
import numpy as np
from logging_setup import log_config  # Auto-configures logging on import
from robot_control import initialize_robot_interfaces
from algebra import coord_sys_math as csm

# Create a logger for this module
logger = logging.getLogger(__name__)

def init_robot():
    rtde_frequency_Hz = 500
    robot_ip = "100.100.100.11"
    rtde_read, rtde_dashboard, rtde_io, rtde_control = initialize_robot_interfaces(rtde_frequency_Hz, robot_ip, logger)
    return rtde_read, rtde_dashboard, rtde_io, rtde_control

def joint_pose(rtde_read, rtde_dashboard, rtde_io, rtde_control ):
    currentJointPose = rtde_read.getActualQ()
    logger.info(f"Current joint pose: {currentJointPose}")
    print(f"Current joint pose: {currentJointPose}")
    return currentJointPose

def tcp_pose(rtde_read, rtde_dashboard, rtde_io, rtde_control):
    currentTcpPose = rtde_read.getActualTCPPose()
    logger.info(f"Current TCP pose: {currentTcpPose}")
    print(f"Current TCP pose: {currentTcpPose}")
    return currentTcpPose

def tcp_offset(rtde_read, rtde_dashboard, rtde_io, rtde_control):
    currentTcp = rtde_control.getTCPOffset()
    logger.info(f"Current TCP: {currentTcp}")
    print(f"Current TCP: {currentTcp}")
    return currentTcp

def measured_points_to_pose_1(top_right: list, top_left: list, bottom_right: list):
    ' top right as origin '
    top_right = np.array(top_right[:3])
    top_left = np.array(top_left[:3])
    bottom_right = np.array(bottom_right[:3])

    x_vector =-1* (bottom_right - top_right)
    y_vector = top_left - top_right
    z_vector = np.cross(x_vector, y_vector)
    y_new_vector = np.cross(z_vector, x_vector)

    x_unit_vector = x_vector / np.linalg.norm(x_vector)
    y_unit_vector = y_new_vector / np.linalg.norm(y_new_vector)
    z_unit_vector = z_vector / np.linalg.norm(z_vector)

    rotation_matrix = np.column_stack((x_unit_vector, y_unit_vector, z_unit_vector))

    axis_angle = csm.convert_rotation_matrix_to_axis_angle_via_quaternion(rotation_matrix, 1e-8)
    rotvec = axis_angle.astype(float)

    poseCoordSystem = (top_right, rotvec)
    print(poseCoordSystem)

def measured_points_to_pose_2(top_right: list, top_left: list, bottom_right: list):
    ' bottom right as origin '
    top_right = np.array(top_right[:3])
    bottom_left = np.array(bottom_left[:3])
    bottom_right = np.array(bottom_right[:3])

    x_vector =-1* (bottom_right - top_right)
    y_vector = bottom_left - bottom_right
    z_vector = np.cross(x_vector, y_vector)
    y_new_vector = np.cross(z_vector, x_vector)

    x_unit_vector = x_vector / np.linalg.norm(x_vector)
    y_unit_vector = y_new_vector / np.linalg.norm(y_new_vector)
    z_unit_vector = z_vector / np.linalg.norm(z_vector)

    rotation_matrix = np.column_stack((x_unit_vector, y_unit_vector, z_unit_vector))

    axis_angle = csm.convert_rotation_matrix_to_axis_angle_via_quaternion(rotation_matrix, 1e-8)
    rotvec = axis_angle.astype(float)

    poseCoordSystem = (top_right, rotvec)
    print(poseCoordSystem)


    

if __name__ == "__main__":
    rtde_read, rtde_dashboard, rtde_io, rtde_control = init_robot()
    joint_pose(rtde_read, rtde_dashboard, rtde_io, rtde_control)
