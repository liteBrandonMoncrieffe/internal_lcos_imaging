import logging
from logging_setup import log_config  # Auto-configures logging on import
from robot_control import initialize_robot_interfaces
# Create a logger for this module
logger = logging.getLogger(__name__)

def joint_pose(rtde_read):
    currentJointPose = rtde_read.getActualQ()
    logger.info(f"Current joint pose: {currentJointPose}")
    return currentJointPose

def tcp_pose(rtde_read):
    currentTcpPose = rtde_read.getActualTCPPose()
    logger.info(f"Current TCP pose: {currentTcpPose}")
    return currentTcpPose

def tcp(rtde_read):
    currentTcp = rtde_read.getActualTCPOffset()
    logger.info(f"Current TCP: {currentTcp}")
    return currentTcp

if __name__ == "__main__":
    # Initialize robot interfaces once
    rtde_frequency_Hz = 500
    robot_ip = "100.100.100.11"
    rtde_read, rtde_dashboard, rtde_io, rtde_control = initialize_robot_interfaces(rtde_frequency_Hz, robot_ip, logger)
    
    tcp(rtde_read)