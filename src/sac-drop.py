import logging
import os
import json
from types import SimpleNamespace
from logging_setup.log_config import setup_logging

import mudi.dispensers
import robot_control
from pypylon import pylon

def json_to_namespace(data):
    """Convert JSON dict to SimpleNamespace for dot notation access."""
    if isinstance(data, dict):
        return SimpleNamespace(**{k: json_to_namespace(v) for k, v in data.items()})
    elif isinstance(data, list):
        return [json_to_namespace(item) for item in data]
    else:
        return data

if __name__ == "__main__":
    tool_center_point_json_path = "configs/tool-center-points.json"
    station_config_json_path = "configs/station-config.json"
    process_config_json_path = "configs/process-config.json"

    connection_and_speeds = json_to_namespace(
        json.load(open(station_config_json_path))
    )
    tool_center_points = json_to_namespace(
        json.load(open(tool_center_point_json_path))
    )
    process_config = json_to_namespace(
        json.load(open(process_config_json_path))
    )

    speed_slider_percentage_decimal = 0.2
    rtde_frequency_Hz = 500
    dispenser_com_port = 8
    dispenser_baud_rate = 57600
    log_file_folder = "C:\\testsw\\robotic_epoxy_squeeze_drop\\logs\\"
    log_file_name = "slide_pnp_log"
    
    # Setup logging with custom config
    log_file_path = os.path.join(log_file_folder, f"{log_file_name}.log")
    os.makedirs(log_file_folder, exist_ok=True)
    setup_logging(log_level=logging.DEBUG, log_file=log_file_path)
    logger = logging.getLogger(__name__)
    
    robot_ip = connection_and_speeds.RobotStation.RobotConfig.RobotIP
    rtde_read, rtde_dashboard, rtde_io, rtde_control = robot_control.initialize_robot_interfaces(rtde_frequency_Hz, robot_ip, logger)
    rtde_io.setSpeedSlider(speed_slider_percentage_decimal)

    dispenser = mudi.dispensers.SuperSigmaCM3()
    dispenser.connect(dispenser_com_port, dispenser_baud_rate)

    picture_save_folder = "C:\\testsw\\robotic_epoxy_squeeze_drop\\dispense_pictures\\"
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    print("Using device:", camera.GetDeviceInfo().GetModelName())

    # get connection info and speeds
    robot_location = connection_and_speeds.RobotStation.RobotConfig.Location
    robot_name = connection_and_speeds.RobotStation.RobotConfig.Name

    joint_speed = connection_and_speeds.RobotStation.RobotDefaultSpeeds.RapidJoint.Velo_rad_per_s
    joint_accel = connection_and_speeds.RobotStation.RobotDefaultSpeeds.RapidJoint.Accel_rad_per_s2
    rapid_linear_speed = connection_and_speeds.RobotStation.RobotDefaultSpeeds.Rapid.Velo_m_per_s
    rapid_linear_accel = connection_and_speeds.RobotStation.RobotDefaultSpeeds.Rapid.Accel_m_per_s2
    touch_linear_speed = connection_and_speeds.RobotStation.RobotDefaultSpeeds.Touch.Velo_m_per_s
    touch_linear_accel = connection_and_speeds.RobotStation.RobotDefaultSpeeds.Touch.Accel_m_per_s2
    fine_touch_speed = connection_and_speeds.RobotStation.RobotDefaultSpeeds.FineTouch.Velo_m_per_s
    fine_touch_accel = connection_and_speeds.RobotStation.RobotDefaultSpeeds.FineTouch.Accel_m_per_s2

    min_area_mm2 = process_config.dispense_analysis.target_area_mm2 - process_config.dispense_analysis.area_tolerance_mm2
    
    # "engineering parameters" ==> hard coded process parameters
    max_tool_z_travel_for_pickup_m = 0.07
    max_tool_z_travel_for_place_m = 0.07
    max_tool_z_travel_for_dispense_m = 0.05
    detection_force_n = 1.2
    squish_force_n = 10.0
    delay_before_vacuum_on_pickup_ms = 1000
    place_delay_before_vac_off_ms = 500
    place_delay_before_retract_s = 4
    slide_pickup_or_place_speed = 0.002
    slide_pickup_or_place_accel = 0.01
    pickup_retract_speed_m_per_s = touch_linear_speed
    pickup_retract_accel_m_per_s2 = touch_linear_accel
    place_retract_speed_m_per_s = touch_linear_speed
    place_retract_accel_m_per_s2 = touch_linear_accel

    # get TCP values
    vacuum_tcp_dict = tool_center_points.slide_vacuum
    syringe_tcp_dict = tool_center_points.sacdrop_syringe
    camera_tcp_dict = tool_center_points.sacdrop_camera
    vacuum_tcp = [
        float(vacuum_tcp_dict.x_m),
        float(vacuum_tcp_dict.y_m),
        float(vacuum_tcp_dict.z_m),
        float(vacuum_tcp_dict.rx_rad),
        float(vacuum_tcp_dict.ry_rad),
        float(vacuum_tcp_dict.rz_rad)
    ]
    syringe_tcp = [
        float(syringe_tcp_dict.x_m),
        float(syringe_tcp_dict.y_m),
        float(syringe_tcp_dict.z_m),
        float(syringe_tcp_dict.rx_rad),
        float(syringe_tcp_dict.ry_rad),
        float(syringe_tcp_dict.rz_rad)
    ]
    camera_tcp = [
        float(camera_tcp_dict.x_m),
        float(camera_tcp_dict.y_m),
        float(camera_tcp_dict.z_m),
        float(camera_tcp_dict.rx_rad),
        float(camera_tcp_dict.ry_rad),
        float(camera_tcp_dict.rz_rad)
    ]

    # get safe positions
    safe_joint_above_pick = process_config.jigs.pick.safe_joint_above
    safe_joint_above_place_location_1 = process_config.jigs.place_location_1.safe_joint_above

    # get pick and place positions
    pick_jig_coords = process_config.jigs.pick.coordinate_system
    place_location_1_coords = process_config.jigs.place_location_1.coordinate_system

    # redefine pick and place poses in base frame
    slide_pickup_base_pose = rtde_control.poseTrans(
        pick_jig_coords,
        process_config.jigs.pick.local_slide_position
    )
    slide_place_1_base_pose = rtde_control.poseTrans(
        place_location_1_coords,
        process_config.jigs.place_location_1.local_slide_position
    )

    # get vacuum IO #
    slide_vacuum_do = process_config.tools.slide_vacuum_do

    # Load parameter sets from file
    param_file = "C:\\testsw\\robotic_epoxy_squeeze_drop\\configs\\dispense_param_sets.json"
    with open(param_file, "r") as f:
        param_sets = json.load(f)

    run_state_file = "run_state.pkl"
    state = load_run_state(run_state_file)
    start_idx = 0
    if state:
        print("Previous run detected.")
        print("Resume previous run? (y/n): ", end="")
        if input().strip().lower() == "y":
            start_idx = state.get("param_idx", 0)
        else:
            os.remove(run_state_file)

    for param_idx, param in enumerate(param_sets[start_idx:], start=start_idx):
        # Get parameter sets for both drops
        param_set_1 = param_sets[param_idx]
        if param_idx + 1 < len(param_sets):
            param_set_2 = param_sets[param_idx + 1]
        else:
            param_set_2 = param_sets[param_idx]  # Repeat last if odd number

        print(f"\n=== Running set {param_idx//2 + 1}: "
            f"Drop 1: time={param_set_1['dispense_time_ms']}ms, pressure={param_set_1['dispense_pressure_kpa']}kPa | "
            f"Drop 2: time={param_set_2['dispense_time_ms']}ms, pressure={param_set_2['dispense_pressure_kpa']}kPa ===")
        
        # load slide
        show_message_box(
            "Load slide into pickup jig, then click OK to continue."
        )

        # go home
        robot_control.go_home(rtde_read, rtde_control, joint_speed, joint_accel, logger)

        rtde_control.setTcp(vacuum_tcp)

        # pick slide
        pickup_location_base_m_rad = pickup_slide(
            rtde_control,
            rtde_read,
            rtde_io,
            logger,
            safe_joint_above_pick,
            slide_pickup_base_pose,
            max_tool_z_travel_for_pickup_m,
            detection_force_n,
            slide_vacuum_do,
            delay_before_vacuum_on_pickup_ms,
            joint_speed,
            joint_accel,
            rapid_linear_speed,
            rapid_linear_accel,
            slide_pickup_or_place_speed,
            slide_pickup_or_place_accel,
            pickup_retract_speed_m_per_s,
            pickup_retract_accel_m_per_s2
        )

        # place slide
        place_location_1_base_m_rad = place_slide(
            rtde_control,
            rtde_read,
            rtde_io,
            logger,
            safe_joint_above_place_location_1,
            slide_place_1_base_pose,
            max_tool_z_travel_for_place_m,
            detection_force_n,
            slide_vacuum_do,
            place_delay_before_vac_off_ms,
            joint_speed,
            joint_accel,
            rapid_linear_speed,
            rapid_linear_accel,
            slide_pickup_or_place_speed,
            slide_pickup_or_place_accel,
            place_retract_speed_m_per_s,
            place_retract_accel_m_per_s2
        )

        # load slide
        show_message_box(
            "Load slide into pickup jig, then click OK to continue."
        )
        
        rtde_control.setTcp(syringe_tcp)
        
        dispense_location_1_points = find_dispense_points(
            rtde_control,
            slide_length_m=0.07526,
            slide_coordinate_system=process_config.jigs.place_location_1.local_slide_position,
            jig_coordinate_system=place_location_1_coords
        )

        safe_joint_above_dispense = process_config.jigs.dispense_location_1.safe_joint_above

        # dispense on slide
        dispense_location_1_base_m_rad, needle_collision = dispense_epoxy(
            rtde_control,
            rtde_read,
            rtde_io,
            logger,
            safe_joint_above_dispense,
            place_location_1_coords,
            dispense_location_1_points,
            max_tool_z_travel_for_dispense_m,
            detection_force_n,
            dispenser,
            [param_set_1, param_set_2],
            joint_speed,
            joint_accel,
            rapid_linear_speed,
            rapid_linear_accel,
            slide_pickup_or_place_speed,
            slide_pickup_or_place_accel,
            place_retract_speed_m_per_s,
            place_retract_accel_m_per_s2
        )

        if not needle_collision:
            rtde_control.setTcp(vacuum_tcp)

            # pick slide 2
            pickup_location_base_m_rad = pickup_slide(
                rtde_control,
                rtde_read,
                rtde_io,
                logger,
                safe_joint_above_pick,
                slide_pickup_base_pose,
                max_tool_z_travel_for_pickup_m,
                detection_force_n,
                slide_vacuum_do,
                delay_before_vacuum_on_pickup_ms,
                joint_speed,
                joint_accel,
                rapid_linear_speed,
                rapid_linear_accel,
                slide_pickup_or_place_speed,
                slide_pickup_or_place_accel,
                pickup_retract_speed_m_per_s,
                pickup_retract_accel_m_per_s2
            )

            slide_2_offset = np.array([0, 0, 0.001, 0, 0, 0])
            local_slide_position = np.array(process_config.jigs.place_location_1.local_slide_position)

            slide_place_2_base_pose = rtde_control.poseTrans(
                place_location_1_coords,
                (local_slide_position + slide_2_offset).tolist()
            )

            # place slide 2
            place_location_2_base_m_rad = place_slide(
                rtde_control,
                rtde_read,
                rtde_io,
                logger,
                safe_joint_above_place_location_1,
                slide_place_2_base_pose,
                max_tool_z_travel_for_place_m,
                squish_force_n,
                slide_vacuum_do,
                place_delay_before_vac_off_ms,
                joint_speed,
                joint_accel,
                rapid_linear_speed,
                rapid_linear_accel,
                slide_pickup_or_place_speed,
                slide_pickup_or_place_accel,
                place_retract_speed_m_per_s,
                place_retract_accel_m_per_s2,
                True
            )

            rtde_control.setTcp(camera_tcp)

            safe_joint_above_camera = process_config.jigs.camera.safe_joint_above

            camera.Open()
            # take pictures of dispense points
            run_id, timestamp = take_pictures(
                rtde_control,
                rtde_read,
                logger,
                camera,
                picture_save_folder,
                dispense_location_1_points,
                safe_joint_above_camera,
                joint_speed_rad_per_s=joint_speed,
                joint_accel_rad_per_s2=joint_accel,
                rapid_speed_m_per_s=rapid_linear_speed,
                rapid_accel_m_per_s2=rapid_linear_accel
            )
            camera.Close()

            # go home
            robot_control.go_home(rtde_read, rtde_control, joint_speed, joint_accel, logger)

            # detect drop areas
            areas = detect_drop_areas(
                picture_save_folder,
                run_id,
                timestamp
            )

            for i, area in enumerate(areas, start=1):
                if isinstance(area, list):
                    if not area:
                        logger.warning(f"Dispense point {i}: no drops detected (empty list).")
                    else:
                        area_str = ", ".join(f"{a:.2f}" for a in area)
                        logger.info(f"Dispense point {i}: detected {len(area)} drop(s) with areas: {area_str} mm^2")
                        for a in area:
                            if a >= min_area_mm2:
                                logger.info(f"  - Area {a:.2f} mm^2 meets threshold")
                            else:
                                logger.warning(f"  - Area {a:.2f} mm^2 below threshold {min_area_mm2} mm^2")
                elif isinstance(area, (int, float)):
                    if area >= min_area_mm2:
                        logger.info(f"Dispense point {i}: detected area ({area:.2f} mm^2) meets threshold")
                    else:
                        logger.warning(f"Dispense point {i}: detected area ({area:.2f} mm^2) below threshold {min_area_mm2} mm^2")
                elif area is None:
                    logger.error(f"Dispense point {i}: area detection failed.")
                else:
                    logger.error(f"Dispense point {i}: unexpected area value type: {type(area)} ({area})")
        
        else:
            logger.error("Needle collision detected during dispense process. Exiting program.")
            break

        # Save progress after each run
        save_run_state({"param_idx": param_idx + 2}, run_state_file)

        print("Run complete. Continue to next set? (y/n): ", end="")
        if input().strip().lower() != "y":
            print("Exiting. Progress saved.")
            break
    
    if not needle_collision:
        print("All parameter sets complete.")
        if os.path.exists(run_state_file):
            os.remove(run_state_file)

    else:
        print("Run ended due to needle collision with cleaning jig. Please address the issue and restart the program.")