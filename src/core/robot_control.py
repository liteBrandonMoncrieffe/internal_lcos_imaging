import numpy as np
import time
from typing import List
import os
import glob
from datetime import datetime
import cv2 as cv

from algebra import plane_math as pm

import robot_control
import mudi  # Only if including dispense_epoxy

def pickup_slide (rtde_control,
                  rtde_read,
                  rtde_IO,
                  logger,
                  safe_joint_near_pickup:List[float],
                  pickup_safe_above_pose:List[float],
                  max_tool_z_travel_for_pickup:float,
                  detection_force:float,
                  robot_digital_out_vacuum:int,
                  delay_before_vacuum_on_ms:int,
                  joint_speed_rad_per_s:float,
                  joint_accel_rad_per_s2:float,
                  rapid_speed_m_per_s:float,
                  rapid_accel_m_per_s2:float,
                  pickup_approach_speed_m_per_s:float,
                  pickup_approach_accel_m_per_s2:float,
                  pickup_retract_speed_m_per_s:float,
                  pickup_retract_accel_m_per_s2:float):

    """
    Move the robot to the pickup position and activate the vacuum.

    **Inputs:**
    - rtde_control: RTDE control interface.
    - rtde_read: RTDE read interface.
    - rtde_IO: RTDE IO interface.
    - logger: Logger for logging messages.
    - safe_joint_near_pickup: Joint angles for a safe position near the pickup location.
    - pickup_safe_above_pose: Pose above the pickup location.
    - max_tool_z_travel_for_pickup: Maximum Z travel for the tool during pickup
    - detection_force: Force threshold for detecting contact.
    - robot_digital_out_vacuum: Digital output number for controlling the vacuum.
    - delay_before_vacuum_on_ms: Delay before turning on the vacuum (in milliseconds).
    - joint_speed_rad_per_s: Joint speed for movements (in rad/s).
    - joint_accel_rad_per_s2: Joint acceleration for movements (in rad/s^2).
    - rapid_speed_m_per_s: Rapid linear speed (in m/s).
    - rapid_accel_m_per_s2: Rapid linear acceleration (in m/s^2).
    - pickup_approach_speed_m_per_s: Speed for approaching the pickup position (in m/s).
    - pickup_approach_accel_m_per_s2: Acceleration for approaching the pickup position (in m/s^2).
    - pickup_retract_speed_m_per_s: Speed for retracting after pickup (in m/s).
    - pickup_retract_accel_m_per_s2: Acceleration for retracting after pickup (in m/s^2).

    **Outputs:**
    - probed_pose: The pose where force contact was detected during pickup.
    """

    rtde_control.moveJ(safe_joint_near_pickup, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    logger.debug(f"Robot moved to safe joint near pickup position:\n\t{safe_joint_near_pickup}")
    rtde_control.moveL(pickup_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
    logger.debug(f"Robot moved to safe pose above pickup position:\n\t{pickup_safe_above_pose}")

    probed_pose = robot_control.force_probe_wrt_tool(
        rtde_control,
        rtde_read,
        rapid_speed_m_per_s=rapid_speed_m_per_s,
        rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
        probe_speed_m_per_s=pickup_approach_speed_m_per_s,
        probe_accel_m_per_sec2=pickup_approach_accel_m_per_s2,
        retract_speed_m_per_s=pickup_retract_speed_m_per_s,
        retract_accel_m_per_sec2=pickup_retract_accel_m_per_s2,
        tool_direction=np.array([0,0,max_tool_z_travel_for_pickup]),
        logger=logger,
        det_force=detection_force,
        retract=False
    )

    logger.info(f"Force contact detected at:\n\t{probed_pose}")
    time.sleep(delay_before_vacuum_on_ms/1000)
    
    rtde_IO.setStandardDigitalOut(robot_digital_out_vacuum, True)
    logger.info(f"Vacuum turned on with DO{robot_digital_out_vacuum}.")
    rtde_control.moveL(pickup_safe_above_pose, pickup_retract_speed_m_per_s, pickup_retract_accel_m_per_s2)
    rtde_control.moveJ(safe_joint_near_pickup, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    
    return probed_pose

def place_slide(rtde_control, 
                rtde_read,
                rtde_IO,
                logger, 
                safe_joint_near_place:List[float],
                place_safe_above_pose:List[float],
                max_tool_z_travel_for_place:float,
                detection_force:float,
                robot_digital_out_vacuum:int,
                delay_before_vacuum_off_ms:int,
                joint_speed_rad_per_s:float,
                joint_accel_rad_per_s2:float,
                rapid_speed_m_per_s:float,
                rapid_accel_m_per_s2:float,
                place_approach_speed_m_per_s:float,
                place_approach_accel_m_per_s2:float,
                place_retract_speed_m_per_s:float,
                place_retract_accel_m_per_s2:float,
                squish_after_place:bool=False):

    """
    Move the robot to the place position and deactivate the vacuum.
    **Inputs:**
    - rtde_control: RTDE control interface.
    - rtde_read: RTDE read interface.
    - rtde_IO: RTDE IO interface.
    - logger: Logger for logging messages.
    - safe_joint_near_place: Joint angles for a safe position near the place location.
    - place_safe_above_pose: Pose above the place location.
    - max_tool_z_travel_for_place: Maximum Z travel for the tool during place
    - detection_force: Force threshold for detecting contact.
    - robot_digital_out_vacuum: Digital output number for controlling the vacuum.
    - delay_before_vacuum_off_ms: Delay before turning off the vacuum (in milliseconds).
    - joint_speed_rad_per_s: Joint speed for movements (in rad/s).
    - joint_accel_rad_per_s2: Joint acceleration for movements (in rad/s^2).
    - rapid_speed_m_per_s: Rapid linear speed (in m/s).
    - rapid_accel_m_per_s2: Rapid linear acceleration (in m/s^2).
    - place_approach_speed_m_per_s: Speed for approaching the place position (in m/s).
    - place_approach_accel_m_per_s2: Acceleration for approaching the place position (in m/s^2).
    - place_retract_speed_m_per_s: Speed for retracting after place (in m/s).
    - place_retract_accel_m_per_s2: Acceleration for retracting after place (in m/s^2).

    **Outputs:**
    - probed_pose: The pose where force contact was detected during place.
    """

    rtde_control.moveJ(safe_joint_near_place, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    logger.info(f"Robot moved to safe joint near place position:\n\t{safe_joint_near_place}")
    rtde_control.moveL(place_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
    logger.info(f"Robot moved to safe pose above place position:\n\t{place_safe_above_pose}")

    probed_pose = robot_control.force_probe_wrt_tool(
        rtde_control,
        rtde_read,
        rapid_speed_m_per_s=rapid_speed_m_per_s,
        rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
        probe_speed_m_per_s=place_approach_speed_m_per_s,
        probe_accel_m_per_sec2=place_approach_accel_m_per_s2,
        retract_speed_m_per_s=place_retract_speed_m_per_s,
        retract_accel_m_per_sec2=place_retract_accel_m_per_s2,
        tool_direction=np.array([0,0,max_tool_z_travel_for_place]),
        logger=logger,
        det_force=detection_force,
        retract=False
        )
    logger.info(f"Force contact detected at:\n\t{probed_pose}")
    if squish_after_place:
        time.sleep(5)
    time.sleep(delay_before_vacuum_off_ms/1000)
    rtde_IO.setStandardDigitalOut(robot_digital_out_vacuum, False)
    logger.info(f"Vacuum turned off with DO{robot_digital_out_vacuum}.")
    time.sleep(0.5)  # Small delay to ensure vacuum is fully released

    logger.info(f"Moving away from place position:\n\tTo: {place_safe_above_pose}")
    rtde_control.moveL(place_safe_above_pose, place_retract_speed_m_per_s, place_retract_accel_m_per_s2)
    rtde_control.moveJ(safe_joint_near_place, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    
    return probed_pose

def dispense_epoxy(rtde_control,
                   rtde_read,
                   rtde_IO,
                   logger,
                   safe_joint_near_dispense:List[float],
                   dispense_jig_coordinate_system:List[float],
                   dispense_points:List[List[float]],
                   max_tool_z_travel_for_dispense:float,
                   detection_force:float,
                   dispenser,
                   param_sets_per_drop:List[dict],
                   joint_speed_rad_per_s:float,
                   joint_accel_rad_per_s2:float,
                   rapid_speed_m_per_s:float,
                   rapid_accel_m_per_s2:float,
                   dispense_approach_speed_m_per_s:float,
                   dispense_approach_accel_m_per_s2:float,
                   dispense_retract_speed_m_per_s:float,
                   dispense_retract_accel_m_per_s2:float,
                   safe_height_above_dispense_m:float=0.004,
                   epoxy_retract_for_dispense_m:float=0.0003):
    
    """
    Dispense epoxy at specified points on the microscope slide.
    **Inputs:**
    - rtde_control: RTDE control interface.
    - rtde_read: RTDE read interface.
    - logger: Logger for logging messages.
    - safe_joint_near_dispense: Joint angles for a safe position near the dispense area.
    - dispense_jig_coordinate_system: Coordinate system of the dispense jig.
    - dispense_points: List of dispense points in base coordinate system.
    - max_tool_z_travel_for_dispense: Maximum Z travel for the tool during dispense.
    - detection_force: Force threshold for detecting contact.
    - dispenser: Dispenser object for controlling epoxy dispensing.
    - epoxy_dispense_time_ms: Time to dispense epoxy (in milliseconds).
    - epoxy_dispense_pressure_kpa: Pressure for dispensing epoxy (in kPa).
    - epoxy_vacuum_pressure_kpa: Vacuum pressure for epoxy drawback (in kPa).
    - joint_speed_rad_per_s: Joint speed for movements (in rad/s).
    - joint_accel_rad_per_s2: Joint acceleration for movements (in rad/s^2).
    - rapid_speed_m_per_s: Rapid linear speed (in m/s).
    - rapid_accel_m_per_s2: Rapid linear acceleration (in m/s^2).
    - dispense_approach_speed_m_per_s: Speed for approaching the dispense position (in m/s).
    - dispense_approach_accel_m_per_s2: Acceleration for approaching the dispense position (in m/s^2).
    - dispense_retract_speed_m_per_s: Speed for retracting after dispense (in m/s).
    - dispense_retract_accel_m_per_s2: Acceleration for retracting after dispense (in m/s^2).
    - safe_height_above_dispense_m: Height above dispense point to move to before/after dispensing (default 0.01 m).
    - epoxy_retract_for_dispense_m: Retract distance from probed point for dispensing (default 0.0003 m).
    
    **Outputs:**
    - probed_poses: List of poses where force contact was detected during dispensing.
    """

    needle_collision = False
    
    # Move to safe joint position near dispense area
    rtde_control.moveJ(safe_joint_near_dispense, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    logger.info(f"Robot moved to safe joint near dispense position:\n\t{safe_joint_near_dispense}")
    
    # Store all probed poses
    probed_poses = []
    
    # Process each dispense point
    for i, dispense_point in enumerate(dispense_points):
        logger.info(f"Processing dispense point {i+1} of {len(dispense_points)}")
       
        params = param_sets_per_drop[i]
        epoxy_dispense_time_ms = params["dispense_time_ms"]
        epoxy_dispense_pressure_kpa = params["dispense_pressure_kpa"]
        epoxy_vacuum_pressure_kpa = params.get("vacuum_pressure_kpa", 2)
        dispenser.set_parameters(epoxy_dispense_pressure_kpa, epoxy_vacuum_pressure_kpa, epoxy_dispense_time_ms)

        # Create safe pose above the dispense point
        probe_plane = coordinate_system_to_plane_parameters(dispense_jig_coordinate_system, logger)
        safe_plane = offset_plane(probe_plane, safe_height_above_dispense_m)
        dispense_safe_above_pose = project_point_onto_plane(safe_plane, dispense_point, logger)

        safe_joint_near_clean = process_config.jigs.needle_cleaner.safe_joint_above
        clean_safe_above_pose = process_config.jigs.needle_cleaner.local_clean_position
        clean_coordinate_system = process_config.jigs.needle_cleaner.coordinate_system
        clean_vacuum_do = process_config.tools.clean_vacuum_do
        
        # Clean the needle before dispensing
        needle_collision = clean_needle(rtde_control,
                     rtde_read,
                     rtde_IO,
                     logger,
                     safe_joint_near_dispense,
                     dispense_safe_above_pose,
                     clean_coordinate_system,
                     clean_safe_above_pose,
                     safe_joint_near_clean,
                     max_tool_z_travel_for_dispense,
                     detection_force,
                     clean_vacuum_do,
                     joint_speed_rad_per_s,
                     joint_accel_rad_per_s2,
                     rapid_speed_m_per_s,
                     rapid_accel_m_per_s2,
                     dispense_approach_speed_m_per_s,
                     dispense_approach_accel_m_per_s2,
                     dispense_retract_speed_m_per_s,
                     dispense_retract_accel_m_per_s2)
        
        if needle_collision:
            logger.warning(f"Needle collision detected during cleaning before dispense point {i+1}. Aborting dispense.")
            break
        
        # 1. Move to above dispense point
        logger.info(f"Moving to above dispense point {i+1}:\n\t{dispense_safe_above_pose}")
        rtde_control.moveL(dispense_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
        
        # 2. Move down and force probe
        logger.info(f"Probing dispense point {i+1}")
        probed_pose = robot_control.force_probe_wrt_tool(
            rtde_control,
            rtde_read,
            rapid_speed_m_per_s=rapid_speed_m_per_s,
            rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
            probe_speed_m_per_s=dispense_approach_speed_m_per_s,
            probe_accel_m_per_sec2=dispense_approach_accel_m_per_s2,
            retract_speed_m_per_s=dispense_retract_speed_m_per_s,
            retract_accel_m_per_sec2=dispense_retract_accel_m_per_s2,
            tool_direction=np.array([0, 0, max_tool_z_travel_for_dispense]),
            logger=logger,
            det_force=detection_force,
            retract=False
        )
        logger.info(f"Force contact detected at point {i+1}:\n\t{probed_pose}")
        probed_poses.append(probed_pose)
        
        # 3. Calculate dispense pose accounting for actual probed height
        # First, find how far the probed point is from the original probe plane
        probe_plane_normal = np.array(probe_plane[:3])
        probe_plane_d = probe_plane[3]
        probed_point = np.array(probed_pose[:3])
        
        # Distance from probed point to probe plane (signed distance)
        distance_to_probe_plane = np.dot(probe_plane_normal, probed_point) + probe_plane_d
        
        logger.info(f"Distance from probed point to probe plane: {distance_to_probe_plane*1000:.3f} mm")
        
        # Calculate total offset needed: distance to probe plane + desired retract amount
        total_offset = distance_to_probe_plane + epoxy_retract_for_dispense_m
        
        # Create dispense plane offset by total amount from probe plane
        dispense_plane = offset_plane(probe_plane, total_offset)
        dispense_pose = project_point_onto_plane(dispense_plane, probed_pose, logger)
        
        logger.info(f"Total dispense offset: {total_offset*1000:.3f} mm ({distance_to_probe_plane*1000:.3f} mm to plane + {epoxy_retract_for_dispense_m*1000:.3f} mm retract)")
        
        # 4. Move to dispense pose and dispense epoxy
        logger.info(f"Moving to dispense at point {i+1}")
        rtde_control.moveL(dispense_pose, dispense_approach_speed_m_per_s, dispense_approach_accel_m_per_s2)

        # Dispense epoxy
        logger.info(f"Dispensing epoxy at point {i+1}")
        dispenser.dispense()
        # Wait for dispense to complete
        time.sleep(epoxy_dispense_time_ms / 1000)
        
        # 5. Retract after dispensing
        logger.info(f"Retracting after dispense at point {i+1}")
        rtde_control.moveL(dispense_safe_above_pose, dispense_retract_speed_m_per_s, dispense_retract_accel_m_per_s2)
    
    if not needle_collision:
        # Move back to safe joint position
        logger.info("All dispense points completed, returning to safe position")
        rtde_control.moveJ(safe_joint_near_dispense, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    dispenser.disconnect()
    
    return probed_poses, needle_collision

def clean_needle(rtde_control,
                 rtde_read,
                 rtde_IO,
                 logger,
                 safe_joint_above_dispense:List[float],
                 dispense_safe_above_pose:List[float],
                 clean_coordinate_system:List[float],
                 local_clean_position:List[float],
                 safe_joint_near_clean:List[float],
                 max_tool_z_travel_for_clean:float,
                 detection_force_n:float,
                 clean_vacuum_do:int,
                 joint_speed_rad_per_s:float,
                 joint_accel_rad_per_s2:float,
                 rapid_speed_m_per_s:float,
                 rapid_accel_m_per_s2:float,
                 clean_approach_speed_m_per_s:float,
                 clean_approach_accel_m_per_s2:float,
                 clean_retract_speed_m_per_s:float,
                 clean_retract_accel_m_per_s2:float):

    """
    Clean the needle using the cleaning jig.

    **Inputs:**
    - rtde_control: RTDE control interface.
    - rtde_read: RTDE read interface.
    - rtde_IO: RTDE IO interface.
    - logger: Logger for logging messages.
    - safe_joint_above_dispense: Joint angles for a safe position above the dispense area.
    - dispense_safe_above_pose: Pose above the dispense location.
    - clean_coordinate_system: Coordinate system of the cleaning jig.
    - local_clean_position: Local position of the cleaning point in the cleaning jig coordinate system.
    - safe_joint_near_clean: Joint angles for a safe position near the cleaning location.
    - max_tool_z_travel_for_clean: Maximum Z travel for the tool during cleaning.
    - detection_force_n: Force threshold for detecting contact during cleaning.
    - clean_vacuum_do: Digital output number for controlling the cleaning vacuum.
    - joint_speed_rad_per_s: Joint speed for movements (in rad/s).
    - joint_accel_rad_per_s2: Joint acceleration for movements (in rad/s^2).
    - rapid_speed_m_per_s: Rapid linear speed (in m/s).
    - rapid_accel_m_per_s2: Rapid linear acceleration (in m/s^2).
    - clean_approach_speed_m_per_s: Speed for approaching the cleaning position (in m/s).
    - clean_approach_accel_m_per_s2: Acceleration for approaching the cleaning position (in m/s^2).
    - clean_retract_speed_m_per_s: Speed for retracting after cleaning (in m/s).
    - clean_retract_accel_m_per_s2: Acceleration for retracting after cleaning (in m/s^2).

    **Outputs:**
    - needle_collision: Boolean indicating if needle collision was detected during intial probe into cleaning jig.
    """

    rtde_control.moveJ(safe_joint_above_dispense, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    logger.info("Moving to clean.")
    rtde_control.moveJ(safe_joint_near_clean, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    clean_safe_above_pose = rtde_control.poseTrans(
        clean_coordinate_system,
        local_clean_position
    )

    rtde_control.moveL(clean_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
    
    # Move down and force probe to above clean position to check for needle alignment
    logger.info("Force probing to clean needle.")
    try:
        probed_pose = robot_control.force_probe_wrt_tool(
            rtde_control,
            rtde_read,
            rapid_speed_m_per_s=rapid_speed_m_per_s,
            rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
            probe_speed_m_per_s=clean_approach_speed_m_per_s,
            probe_accel_m_per_sec2=clean_approach_accel_m_per_s2,
            retract_speed_m_per_s=clean_retract_speed_m_per_s,
            retract_accel_m_per_sec2=clean_retract_accel_m_per_s2,
            tool_direction=np.array([0,0,local_clean_position[2]]),
            logger=logger,
            det_force=detection_force_n,
            retract=False
        )
    except robot_control.routines.ForceProbeError:
        probed_pose = None

    if probed_pose is None:
        needle_collision = False
        logger.info("No force contact detected.")

        clean_inserted_pose = (np.array(clean_safe_above_pose) - np.array([0,0,0.026,0,0,0])).tolist()

        # Move to almost bottom of cleaner
        rtde_control.moveL(clean_inserted_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)

        # Force probe to bottom of cleaner
        probed_pose = robot_control.force_probe_wrt_tool(
            rtde_control,
            rtde_read,
            rapid_speed_m_per_s=rapid_speed_m_per_s,
            rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
            probe_speed_m_per_s=clean_approach_speed_m_per_s,
            probe_accel_m_per_sec2=clean_approach_accel_m_per_s2,
            retract_speed_m_per_s=clean_retract_speed_m_per_s,
            retract_accel_m_per_sec2=clean_retract_accel_m_per_s2,
            tool_direction=np.array([0,0,0.002]),
            logger=logger,
            det_force=detection_force_n,
            retract=False
        )

        # Clean needle
        rtde_IO.setStandardDigitalOut(clean_vacuum_do, True)
        logger.info(f"Cleaning vacuum turned on with DO{clean_vacuum_do}.")
        time.sleep(1)  # Wait for cleaning action
        rtde_IO.setStandardDigitalOut(clean_vacuum_do, False)
        logger.info(f"Cleaning vacuum turned off with DO{clean_vacuum_do}.")

        # Retract from cleaner
        rtde_control.moveL(clean_safe_above_pose, clean_retract_speed_m_per_s, clean_retract_accel_m_per_s2)
        rtde_control.moveJ(safe_joint_near_clean, joint_speed_rad_per_s, joint_accel_rad_per_s2)

        # Return to dispense position
        rtde_control.moveJ(safe_joint_above_dispense, joint_speed_rad_per_s, joint_accel_rad_per_s2)
        rtde_control.moveL(dispense_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
    else:
        needle_collision = True
        logger.error(f"Force contact detected at:\n\t{probed_pose}. Needle is not aligned properly!")
        rtde_control.moveL(clean_safe_above_pose, clean_retract_speed_m_per_s, clean_retract_accel_m_per_s2)
        rtde_control.moveJ(safe_joint_near_clean, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    return needle_collision


def take_pictures(rtde_control,
                  rtde_read,
                  logger,
                  camera,
                  picture_save_folder:str,
                  dispense_points:List[List[float]],
                  safe_joint_near_picture:List[float],
                  joint_speed_rad_per_s:float,
                  joint_accel_rad_per_s2:float,
                  rapid_speed_m_per_s:float,
                  rapid_accel_m_per_s2:float):

    """
    Take pictures at specified dispense points and save them with unique IDs and timestamps.

    **Inputs:**
    - rtde_control: RTDE control interface.
    - rtde_read: RTDE read interface.
    - logger: Logger for logging messages.
    - camera: Camera object for taking pictures.
    - picture_save_folder: Folder path to save pictures.
    - dispense_points: List of dispense points in base coordinate system.
    - safe_joint_near_picture: Joint angles for a safe position near the picture area.
    - joint_speed_rad_per_s: Joint speed for movements (in rad/s).
    - joint_accel_rad_per_s2: Joint acceleration for movements (in rad/s^2).
    - rapid_speed_m_per_s: Rapid linear speed (in m/s).
    - rapid_accel_m_per_s2: Rapid linear acceleration (in m/s^2).
    
    **Outputs:**
    - next_id: The next available ID number used in filenames.
    - timestamp: The timestamp used in filenames.
    """
    
    # Find the next available ID number
    existing_files = glob.glob(os.path.join(picture_save_folder, "dispense_point_*_*-*.png"))
    existing_ids = []
    for f in existing_files:
        try:
            base = os.path.basename(f)
            # Extract the ID between the second underscore and the dash
            # Example: dispense_point_2_5-20241112_150701.png -> Y=5
            id_part = base.split("_")[3].split("-")[0]
            existing_ids.append(int(id_part))
        except Exception:
            continue
    next_id = max(existing_ids) + 1 if existing_ids else 1

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    rtde_control.moveJ(safe_joint_near_picture, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    logger.info(f"Robot moved to safe joint near picture position:\n\t{safe_joint_near_picture}")

    for i, dispense_point in enumerate(dispense_points):
        rtde_control.moveL(dispense_point, rapid_speed_m_per_s, rapid_accel_m_per_s2)
        logger.info(f"Robot moved to picture position {i+1}:\n\t{dispense_point}")

        if i == 0:
            time.sleep(2)  # Extra delay for first position to allow camera to adjust
            # Set camera settings for first picture
            camera.ExposureAuto.SetValue('Once')
            camera.GainAuto.SetValue('Once')
        time.sleep(1) # Small delay to allow for any vibrations to settle
       
   
    rtde_control.moveJ(safe_joint_near_picture, joint_speed_rad_per_s, joint_accel_rad_per_s2)
    
    return next_id, timestamp

def find_dispense_points(rtde_control,
                         slide_length_m:float,
                         slide_coordinate_system:List[float],
                         jig_coordinate_system:List[float]) -> List[List[float]]:
    
    """
    Generate dispense points along the length of the slide in the jig coordinate system.
    
    **Inputs:**
    - rtde_control: RTDE control interface.
    - slide_length_m: Length of the microscope slide in meters.
    - slide_coordinate_system: Coordinate system of the slide.
    - jig_coordinate_system: Coordinate system of the jig.

    **Outputs:**
    - base_dispense_points: List of dispense points in the base coordinate system.
    """

    # Configure this to set dispense points. (0,0,0) is the center of the slide, where the Y axis is along the length.
    slide_dispense_points = [(0, 0, 0, 0, 0, 0)]

    jig_dispense_points = []

    for i, offset in enumerate(slide_dispense_points):
        jig_dispense_points.append(rtde_control.poseTrans(
            slide_coordinate_system,
            offset
        ))

    base_dispense_points = []

    for i, offset in enumerate(jig_dispense_points):
        base_dispense_points.append(rtde_control.poseTrans(
            jig_coordinate_system,
            offset
        ))

    return base_dispense_points

def clean_needle(rtde_control,
                 rtde_read,
                 rtde_IO,
                 logger,
                 safe_joint_above_dispense:List[float],
                 dispense_safe_above_pose:List[float],
                 clean_coordinate_system:List[float],
                 local_clean_position:List[float],
                 safe_joint_near_clean:List[float],
                 max_tool_z_travel_for_clean:float,
                 detection_force_n:float,
                 clean_vacuum_do:int,
                 joint_speed_rad_per_s:float,
                 joint_accel_rad_per_s2:float,
                 rapid_speed_m_per_s:float,
                 rapid_accel_m_per_s2:float,
                 clean_approach_speed_m_per_s:float,
                 clean_approach_accel_m_per_s2:float,
                 clean_retract_speed_m_per_s:float,
                 clean_retract_accel_m_per_s2:float):

    """
    Clean the needle using the cleaning jig.

    **Inputs:**
    - rtde_control: RTDE control interface.
    - rtde_read: RTDE read interface.
    - rtde_IO: RTDE IO interface.
    - logger: Logger for logging messages.
    - safe_joint_above_dispense: Joint angles for a safe position above the dispense area.
    - dispense_safe_above_pose: Pose above the dispense location.
    - clean_coordinate_system: Coordinate system of the cleaning jig.
    - local_clean_position: Local position of the cleaning point in the cleaning jig coordinate system.
    - safe_joint_near_clean: Joint angles for a safe position near the cleaning location.
    - max_tool_z_travel_for_clean: Maximum Z travel for the tool during cleaning.
    - detection_force_n: Force threshold for detecting contact during cleaning.
    - clean_vacuum_do: Digital output number for controlling the cleaning vacuum.
    - joint_speed_rad_per_s: Joint speed for movements (in rad/s).
    - joint_accel_rad_per_s2: Joint acceleration for movements (in rad/s^2).
    - rapid_speed_m_per_s: Rapid linear speed (in m/s).
    - rapid_accel_m_per_s2: Rapid linear acceleration (in m/s^2).
    - clean_approach_speed_m_per_s: Speed for approaching the cleaning position (in m/s).
    - clean_approach_accel_m_per_s2: Acceleration for approaching the cleaning position (in m/s^2).
    - clean_retract_speed_m_per_s: Speed for retracting after cleaning (in m/s).
    - clean_retract_accel_m_per_s2: Acceleration for retracting after cleaning (in m/s^2).

    **Outputs:**
    - needle_collision: Boolean indicating if needle collision was detected during intial probe into cleaning jig.
    """

    rtde_control.moveJ(safe_joint_above_dispense, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    logger.info("Moving to clean.")
    rtde_control.moveJ(safe_joint_near_clean, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    clean_safe_above_pose = rtde_control.poseTrans(
        clean_coordinate_system,
        local_clean_position
    )

    rtde_control.moveL(clean_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
    
    # Move down and force probe to above clean position to check for needle alignment
    logger.info("Force probing to clean needle.")
    try:
        probed_pose = robot_control.force_probe_wrt_tool(
            rtde_control,
            rtde_read,
            rapid_speed_m_per_s=rapid_speed_m_per_s,
            rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
            probe_speed_m_per_s=clean_approach_speed_m_per_s,
            probe_accel_m_per_sec2=clean_approach_accel_m_per_s2,
            retract_speed_m_per_s=clean_retract_speed_m_per_s,
            retract_accel_m_per_sec2=clean_retract_accel_m_per_s2,
            tool_direction=np.array([0,0,local_clean_position[2]]),
            logger=logger,
            det_force=detection_force_n,
            retract=False
        )
    except robot_control.routines.ForceProbeError:
        probed_pose = None

    if probed_pose is None:
        needle_collision = False
        logger.info("No force contact detected.")

        clean_inserted_pose = (np.array(clean_safe_above_pose) - np.array([0,0,0.026,0,0,0])).tolist()

        # Move to almost bottom of cleaner
        rtde_control.moveL(clean_inserted_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)

        # Force probe to bottom of cleaner
        probed_pose = robot_control.force_probe_wrt_tool(
            rtde_control,
            rtde_read,
            rapid_speed_m_per_s=rapid_speed_m_per_s,
            rapid_accel_m_per_sec2=rapid_accel_m_per_s2,
            probe_speed_m_per_s=clean_approach_speed_m_per_s,
            probe_accel_m_per_sec2=clean_approach_accel_m_per_s2,
            retract_speed_m_per_s=clean_retract_speed_m_per_s,
            retract_accel_m_per_sec2=clean_retract_accel_m_per_s2,
            tool_direction=np.array([0,0,0.002]),
            logger=logger,
            det_force=detection_force_n,
            retract=False
        )

        # Clean needle
        rtde_IO.setStandardDigitalOut(clean_vacuum_do, True)
        logger.info(f"Cleaning vacuum turned on with DO{clean_vacuum_do}.")
        time.sleep(1)  # Wait for cleaning action
        rtde_IO.setStandardDigitalOut(clean_vacuum_do, False)
        logger.info(f"Cleaning vacuum turned off with DO{clean_vacuum_do}.")

        # Retract from cleaner
        rtde_control.moveL(clean_safe_above_pose, clean_retract_speed_m_per_s, clean_retract_accel_m_per_s2)
        rtde_control.moveJ(safe_joint_near_clean, joint_speed_rad_per_s, joint_accel_rad_per_s2)

        # Return to dispense position
        rtde_control.moveJ(safe_joint_above_dispense, joint_speed_rad_per_s, joint_accel_rad_per_s2)
        rtde_control.moveL(dispense_safe_above_pose, rapid_speed_m_per_s, rapid_accel_m_per_s2)
    else:
        needle_collision = True
        logger.error(f"Force contact detected at:\n\t{probed_pose}. Needle is not aligned properly!")
        rtde_control.moveL(clean_safe_above_pose, clean_retract_speed_m_per_s, clean_retract_accel_m_per_s2)
        rtde_control.moveJ(safe_joint_near_clean, joint_speed_rad_per_s, joint_accel_rad_per_s2)

    return needle_collision