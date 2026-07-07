# testing
import cv2
import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import (
    start_vision_system,
    start_web_server,
    get_latest_data,
    stop_vision_system,
    get_track_distance,
    add_marker_point,        
    remove_marker_point,
    clear_marker_points,
    set_color_detection,
    set_all_color_detection,
    configure_vision_pipeline
)
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")
esp = ESP32Communicator()
esp.connect()
camera = picam2()
video_size = (400, 225)
sensor_video_size = (2304, 1296)
target_frame_duration_us = 10000

config = camera.create_video_configuration(
        main={"size": video_size, "format": "BGR888"},
        sensor={"output_size": sensor_video_size},
        controls={"FrameDurationLimits": (target_frame_duration_us, target_frame_duration_us)},
        buffer_count=4,
        queue=False,
)
camera.configure(config)
camera.start()
print("Camera: Activated")

record_mp4 = True
live_streaming = True

configure_vision_pipeline(
    draw_overlays=True,
    show_debug_strip=False,
    stream_use_debug_frame=False,
    record_use_debug_frame=False,
    stream_jpeg_quality=70,
)

start_vision_system(camera, record_mp4)

if live_streaming:
        print("Streaming: Activated")
        start_web_server(host='0.0.0.0', port=5000)
else:
        print("Streaming: Not streaming")

print("======= End of Init =======")

def esp_replyNprint():
        try:
                reply = esp.read_message()
                if reply is not None:
                        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        print(f"{timestamp} ESP: {reply}")
                        return reply
        except Exception as e:
                print(f"\n[SERIAL WARNING] Caught hardware communication glitch: {e}")
                print("Attempting to bypass frame drop...")
        return None

last_action_logged = ""
def send_command_logged(cmd_str):
        global last_action_logged
        esp.send_command(cmd_str)
        if cmd_str != last_action_logged:
                print(f"[ESP_OUT] Sent Command: {cmd_str}")
                last_action_logged = cmd_str

def stop_vehicle(reason):
        print(f"[STOP] {reason}")
        send_command_logged("0, 0, 0, motor stop")

def add_marker_point_wrapper(name, x, y, color=(0, 255, 255), radius=5, label=None):
        add_marker_point(name, x, y, color=color, radius=radius, label=label)

def remove_marker(name):
    remove_marker_point(name)

def set_red_detection(enabled):
    set_color_detection("red", enabled)

def set_green_detection(enabled):
    set_color_detection("green", enabled)

def set_magenta_detection(enabled):
    set_color_detection("magenta", enabled)

def set_color_block_detection(red=None, green=None, magenta=None):
    set_all_color_detection(red=red, green=green, magenta=magenta, white=True)

def set_debug_mode(enabled):
    if enabled:
        configure_vision_pipeline(
            draw_overlays=True,
            show_debug_strip=True,
            stream_use_debug_frame=True,
            record_use_debug_frame=True,
            stream_jpeg_quality=85,
        )
    else:
        configure_vision_pipeline(
            draw_overlays=True,
            show_debug_strip=False,
            stream_use_debug_frame=False,
            record_use_debug_frame=False,
            stream_jpeg_quality=70,
        )

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def blend(start, end, progress):
    progress = clamp(progress, 0.0, 1.0)
    return start + (end - start) * progress

def find_track_edge_x(track_polygon, row_y, scan_from_left, coarse_step=6):
    if track_polygon is None:
        return None

    frame_width = video_size[0]
    if scan_from_left:
        x_range = range(0, frame_width, coarse_step)
        refine_direction = -1
    else:
        x_range = range(frame_width - 1, -1, -coarse_step)
        refine_direction = 1

    candidate = None
    for x in x_range:
        if cv2.pointPolygonTest(track_polygon, (float(x), float(row_y)), False) >= 0:
            candidate = x
            break

    if candidate is None:
        return None

    refine_x = candidate
    while 0 <= refine_x + refine_direction < frame_width:
        next_x = refine_x + refine_direction
        if cv2.pointPolygonTest(track_polygon, (float(next_x), float(row_y)), False) >= 0:
            refine_x = next_x
        else:
            break
    return refine_x

def measure_track_profile(track_polygon, sample_rows):
    left_hits = []
    right_hits = []

    for row_y in sample_rows:
        left_x = find_track_edge_x(track_polygon, row_y, scan_from_left=True)
        right_x = find_track_edge_x(track_polygon, row_y, scan_from_left=False)
        if left_x is None or right_x is None:
            continue
        if right_x - left_x < 40:
            continue
        left_hits.append(left_x)
        right_hits.append(right_x)

    if not left_hits:
        return None

    avg_left = sum(left_hits) / len(left_hits)
    avg_right = sum(right_hits) / len(right_hits)
    width = avg_right - avg_left
    return {
        "left_x": avg_left,
        "right_x": avg_right,
        "width": width,
    }

def compute_wall_follow_steering(track_polygon, is_clockwise, previous_error, recovery_mode=False, target_ratio_override=None):
    sample_rows = (120, 140, 160) if recovery_mode else (170, 185, 200)
    profile = measure_track_profile(track_polygon, sample_rows)
    if profile is None:
        return 0, previous_error, None

    target_ratio = target_ratio_override if target_ratio_override is not None else (0.6 if is_clockwise else 0.4)
    target_x = profile["left_x"] + profile["width"] * target_ratio
    error = target_x - (video_size[0] / 2)
    error_delta = error - previous_error

    if recovery_mode:
        kp = 1.2
        kd = 0.7
        max_steer = 55
    else:
        kp = 0.55
        kd = 0.65
        max_steer = 65

    steering = (kp * error) + (kd * error_delta)
    steering = int(round(clamp(steering, -max_steer, max_steer)))
    profile["target_x"] = target_x
    profile["error"] = error
    profile["target_ratio"] = target_ratio
    return steering, error, profile

def get_sector_target_ratio(is_clockwise, completed_turns):
    start_ratio = 0.52 if is_clockwise else 0.48
    final_ratio = 0.58 if is_clockwise else 0.42
    progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
    return blend(start_ratio, final_ratio, progress)

def get_sector_turn_duration_ms(completed_turns):
    start_duration_ms = 1000
    min_duration_ms = 750
    progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
    return blend(start_duration_ms, min_duration_ms, progress)

def calculate_oc2_steering_bias(track_data, obstacle_data, current_angle):
        if not hasattr(calculate_oc2_steering_bias, "was_dodging"):
                calculate_oc2_steering_bias.was_dodging = False
                calculate_oc2_steering_bias.last_dodge_dir = None

        target_angle = current_angle

        base_kick = 35.0  
        gain_multiplier = 0.9
        max_clamp = 85.0
        deadzone_weigh = 3.0

        COUNTER_STEER_FORCE = 25.0
        pillar_detected_this_frame = False

        if obstacle_data and obstacle_data.get("color") in ["RED", "GREEN"]:
                cx = obstacle_data["center_x"]
                obstacle_area = (obstacle_data["width"] * obstacle_data["height"]) // 100

                if (0 <= cx <= deadzone_weigh) or (video_size[0] - deadzone_weigh <= cx <= video_size[0]):
                        print(f"[FILTER_IGNORE] Spotted {obstacle_data['color']} pillar at boundary edge (X: {cx}). Skipping processing.")
                        return target_angle

                print(f"[OBSTACLE_ALERT] Mid-lane Pillar Active! Color: {obstacle_data['color']} | Center X: {cx} | Area: {obstacle_area}")

                if obstacle_area > 5: 
                        pillar_detected_this_frame = True
                        dynamic_offset = base_kick + (obstacle_area * gain_multiplier)
                        dynamic_offset = min(dynamic_offset, max_clamp)

                        if obstacle_data["color"] == "RED":
                                target_angle += dynamic_offset
                                calculate_oc2_steering_bias.was_dodging = True
                                calculate_oc2_steering_bias.last_dodge_dir = "RIGHT"
                                print(f"[AVOIDANCE_ACTION!!] RED Pillar. Steering Bias RIGHT (+{dynamic_offset:.2f}).")
                        elif obstacle_data["color"] == "GREEN":
                                target_angle -= dynamic_offset
                                calculate_oc2_steering_bias.was_dodging = True
                                calculate_oc2_steering_bias.last_dodge_dir = "LEFT"
                                print(f"[AVOIDANCE_ACTION!!] GREEN Pillar. Steering Bias LEFT (-{dynamic_offset:.2f}).")
                else:
                        print("[AVOIDANCE_SKIP] Distance gap safe. Skipping bias modifier adjustment.")

        if not pillar_detected_this_frame and calculate_oc2_steering_bias.was_dodging:
                print(f"[COUNTER-STEER RECOVERY] Pillar cleared! Executing stabilization correction.")

                if calculate_oc2_steering_bias.last_dodge_dir == "RIGHT":
                        recovery_angle = -COUNTER_STEER_FORCE
                        target_angle += recovery_angle
                        print(f" -> counter action: Pulsing LEFT ({recovery_angle}) to catch alignment.")
                else:
                        recovery_angle = COUNTER_STEER_FORCE
                        target_angle += recovery_angle
                        print(f" -> counter action: Pulsing RIGHT (+{recovery_angle}) to catch alignment.")

                calculate_oc2_steering_bias.was_dodging = False
                calculate_oc2_steering_bias.last_dodge_dir = None

        return int(round(clamp(target_angle, -max_clamp, max_clamp)))

# Checkpoints (default as clockwise case)
front_point = [200, 80]
add_marker_point_wrapper("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
left_turning_point = [5, 200]
add_marker_point_wrapper("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
right_turning_point = [395, 200]
add_marker_point_wrapper("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
clockwise_indicator = [395, 100]
add_marker_point_wrapper("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
anticlockwise_indicator = [5, 100]
add_marker_point_wrapper("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")

turning_point = right_turning_point
track_left = [375, 225]
track_right = [400, 225]
ending_point = [200, 62.5]

# Purple Wall Detector Config & Anti-Double-Count Time Locks
PURPLE_LAP_BUFFER = 3.0
MIN_PURPLE_AREA = 25

# Global variables
run_OC1 = False
run_OC2 = False
reset_OC = True
num_of_turn = 0
is_clockwise = True
previous_wall_error = 0.0
dash_start_time = 0.0
start_time = 0
end_time = 0
recorded_time = 0

# OC1 States
class States(Enum):
    INIT = 0
    FIRST_SECTOR = 1
    WAIT_TURN_STATE = 2
    TURNING_STATE = 3
    DASH_AFTER_TURNING_STATE = 4
    RUN_SECTOR_STATE = 5
    LAST_RUN = 6

state = States.INIT
last_state = States.INIT
speed = 10

try:
        print("IDLE")
        while True:
                if reset_OC:
                        if run_OC1 or run_OC2:
                                angle = 0
                                turn_time = 0
                                num_of_turn = 0
                                lap_count_oc1 = 0
                                last_purple_lap_time = 0.0
                                purple_wall_lockout_active = False
                                is_clockwise = True
                                previous_wall_error = 0.0
                                dash_start_time = 0.0
                                turning_point = right_turning_point
                                state = States.FIRST_SECTOR
                                last_state = None

                                if hasattr(calculate_oc2_steering_bias, "was_dodging"):
                                        calculate_oc2_steering_bias.was_dodging = False
                                        calculate_oc2_steering_bias.last_dodge_dir = None

                                clear_marker_points()
                                # Checkpoints (default as clockwise case)
                                front_point = [200, 80]
                                add_marker_point_wrapper("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
                                left_turning_point = [5, 200]
                                add_marker_point_wrapper("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
                                right_turning_point = [395, 200]
                                add_marker_point_wrapper("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
                                clockwise_indicator = [395, 100]
                                add_marker_point_wrapper("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
                                anticlockwise_indicator = [5, 100]
                                add_marker_point_wrapper("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")

                                print("Reset Complete. state = FIRST_SECTOR")
                        reset_OC = False
                        continue

                # ======================================================================
                # RUN OC1 BLOCK WITH INTEGRATED OC2 LOGIC
                # ======================================================================
                elif run_OC1:
                        send_command_logged("OC1")
                        start_time = time.perf_counter()
                        while run_OC1:
                                current_frame_time = time.perf_counter()
                                reply = esp_replyNprint()
                                if reply == "EOC1":
                                        print(f"\n[{state.name}][FSM ALERT] EOC1 Signal Received from ESP32. Terminating run immediately.")
                                        stop_vehicle("OC1 stop requested by ESP32")
                                        run_OC1 = False
                                        break

                                set_color_block_detection(red=True, green=True, magenta=True)
                                obstacle, parking, track = get_latest_data()

                                # --- Option A Integration for OC1 Loop ---
                                if obstacle["color"] in ["RED", "GREEN"]:
                                        print(f"[DETECT][OC1] {obstacle['color']} pillar spotted! Center X: {obstacle['center_x']}, Y: {obstacle['center_y']} | Width: {obstacle['width']}, Height: {obstacle['height']}")

                                if state != last_state:
                                        print(f"\n==================================================")
                                        print(f"[{state.name}][FSM CHANGE] State Transition: {last_state} ===> {state}")
                                        print(f"==================================================\n")
                                        last_state = state

                                # --------------------------------------------------------------
                                # DYNAMIC LOCKOUT LOGIC: PURPLE WALL RANGE BOUNDARY WINDOW
                                # --------------------------------------------------------------
                                purple_area_check = (parking["width"] * parking["height"]) // 100

                                # Condition A: Purple wall is scanned on screen
                                if parking["center_x"] != 0 and purple_area_check >= MIN_PURPLE_AREA:
                                        if not purple_wall_lockout_active:
                                                print(f"[{state.name}][LOCKOUT ENGAGED] Purple wall entered frame. Pillar detection completely stopped.")
                                                purple_wall_lockout_active = True

                                # Condition B: Purple wall has completely gone out of the screen
                                elif parking["center_x"] == 0:
                                        if purple_wall_lockout_active:
                                                print(f"[{state.name}][LOCKOUT RELEASED] Purple wall went completely off screen. Restoring pillar detection.")
                                                purple_wall_lockout_active = False

                                # If the lockout state is active, suppress all pillar calculations
                                if purple_wall_lockout_active and obstacle:
                                        obstacle["color"] = None

                                if state == States.FIRST_SECTOR:
                                    # FIXED: Run the 0 base angle through the obstacle calculation function instead of a hardcoded 0
                                    first_sector_steering = calculate_oc2_steering_bias(track, obstacle, 0)
                                    esp.send_command(str(speed) + ", " + str(first_sector_steering) + ", -1, forward")
                                    print(f"[{state.name}][MOVE] Sent forward command: Speed={speed}, Steering={first_sector_steering}")
                                    
                                    if get_track_distance(front_point[0], front_point[1])[0] == False and (get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True or get_track_distance(right_turning_point[0], right_turning_point[1])[0] == True):
                                        if num_of_turn == 0:
                                            if get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]:
                                                is_clockwise = True
                                                remove_marker("Anticlockwise Indicator")
                                                innerwall_white = [335, 220]
                                                innerwall_black = [365, 220]
                                            else:
                                                is_clockwise = False
                                                remove_marker("Clockwise Indicator")
                                                innerwall_white = [65, 220]
                                                innerwall_black = [35, 220]
                                            add_marker_point_wrapper("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
                                            add_marker_point_wrapper("Innerwall White", innerwall_white[0], innerwall_white[1], color=(0, 0, 255), radius=3, label="Danger")
                                            add_marker_point_wrapper("Innerwall Black", innerwall_black[0], innerwall_black[1], color=(255, 0, 0), radius=3, label="Safe")
                                            is_clockwise = get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]
                                            previous_wall_error = 0.0
                                        start_turning_time = time.perf_counter_ns()
                                        state = States.TURNING_STATE
                                        continue
                                    else:
                                            time.sleep(0.001)
                                            continue

                                # Turn
                                elif state == States.TURNING_STATE:
                                    turn_elapsed_ms = (time.perf_counter_ns() - start_turning_time) / 1000000
                                    blind_turn_duration_ms = get_sector_turn_duration_ms(num_of_turn)
                                    if turn_elapsed_ms < blind_turn_duration_ms:
                                        steering = 80 if is_clockwise else -80
                                        esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn-in")
                                        print(f"[{state.name}][MOVE][BLIND-TURN] Sent turn-in command: Speed={speed}, Steering={steering} (Elapsed: {turn_elapsed_ms:.1f}ms / Blind Target: {blind_turn_duration_ms:.1f}ms)")
                                        continue

                                    steering, previous_wall_error, profile = compute_wall_follow_steering(
                                        track["polygon"],
                                        is_clockwise,
                                        previous_wall_error,
                                        recovery_mode=True,
                                        target_ratio_override=(0.52 if is_clockwise else 0.48),
                                    )
                                    if profile is None:
                                        steering = 55 if is_clockwise else -55
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn-align")
                                    print(f"[{state.name}][MOVE][ALIGN-TURN] Sent turn-align command: Speed={speed}, Steering={steering} (Error Profile: {None if profile is None else profile.get('error')})")

                                    if turn_elapsed_ms < 1100:
                                        continue

                                    if profile is not None and abs(profile["error"]) <= 18:
                                        num_of_turn += 1
                                        print("num turn: {}".format(num_of_turn))
                                        dash_start_time = time.perf_counter()
                                        state = States.DASH_AFTER_TURNING_STATE if num_of_turn < 12 else States.LAST_RUN
                                        continue

                                    if turn_elapsed_ms < 1500:
                                        continue

                                    num_of_turn += 1
                                    print("num turn: {}".format(num_of_turn))
                                    dash_start_time = time.perf_counter()
                                    state = States.DASH_AFTER_TURNING_STATE if num_of_turn < 12 else States.LAST_RUN
                                    continue

                                elif state == States.DASH_AFTER_TURNING_STATE:
                                    target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                    steering, previous_wall_error, _ = compute_wall_follow_steering(
                                        track["polygon"],
                                        is_clockwise,
                                        previous_wall_error,
                                        target_ratio_override=target_ratio,
                                    )
                                    if is_clockwise:
                                        steering = steering + 30
                                    else:
                                        steering = steering - 30
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, settle after turn")
                                    print(f"[{state.name}][MOVE] Sent settle after turn command: Speed={speed}, Steering={steering}, Target Ratio={target_ratio:.3f}")
                                    if time.perf_counter() - dash_start_time < 0.35:
                                        continue
                                    state = States.RUN_SECTOR_STATE
                                    continue

                                # Run sector and keep a certain distance from the inner barrier
                                elif state == States.RUN_SECTOR_STATE:
                                    if get_track_distance(front_point[0], front_point[1])[0] == False and (get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True or get_track_distance(right_turning_point[0], right_turning_point[1])[0] == True):
                                            state = States.TURNING_STATE
                                            previous_wall_error = 0.0
                                            start_turning_time = time.perf_counter_ns()
                                            continue
                                    target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                    steering, previous_wall_error, profile = compute_wall_follow_steering(
                                        track["polygon"],
                                        is_clockwise,
                                        previous_wall_error,
                                        target_ratio_override=target_ratio,
                                    )
                                    if profile is None:
                                        if not get_track_distance(innerwall_white[0], innerwall_white[1])[0]:
                                            steering = -50 if is_clockwise else 50
                                        elif get_track_distance(innerwall_black[0], innerwall_black[1])[0]:
                                            steering = 5 if is_clockwise else -5
                                        else:
                                            steering = 0

                                    # Modified steering angle calculations based on dynamic obstacle modifiers
                                    adjusted_steering = calculate_oc2_steering_bias(track, obstacle, steering)
                                    print(f"[{state.name}][STEERING_DECISION] Cruise Sector Output -> Speed: {speed} | Base Wall Steering: {steering} | Combined Adjusted: {adjusted_steering}")

                                    # ---- MAGENTA CHROMATIC OVERLAY LAP COUNTER ----
                                    purple_x = parking["center_x"]
                                    purple_area = (parking["width"] * parking["height"]) // 100

                                    if purple_x != 0 and purple_area >= MIN_PURPLE_AREA:
                                        time_since_last_lap = current_frame_time - last_purple_lap_time

                                        if time_since_last_lap >= PURPLE_LAP_BUFFER:
                                            lap_count_oc1 += 1
                                            last_purple_lap_time = current_frame_time

                                            print(f"\n🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮")
                                            print(f"[{state.name}][LAP REGISTERED] Magenta wall verified! Loop Count: [ {lap_count_oc1} / 3 ]")
                                            print(f"[{state.name}][TIME LOCK] Guard frame window active for {PURPLE_LAP_BUFFER} seconds.")
                                            print(f"🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮\n")

                                            if lap_count_oc1 >= 3:
                                                print(f"[{state.name}][OC1 METRIC PASSED] Target loop requirement achieved. Swapping to stop sequence.")
                                                state = States.LAST_RUN
                                                continue
                                        else:
                                                remaining_lockout = PURPLE_LAP_BUFFER - time_since_last_lap
                                                print(f"[{state.name}][LAP BLOCK] Magenta marker active, but locked in debounce window. Shield: {remaining_lockout:.2f}s.")

                                    final_steering = int(round(adjusted_steering))
                                    esp.send_command(str(speed) + ", " + str(final_steering) + ", -1, wall follow")
                                    print(f"[{state.name}][MOVE] Sent wall follow command: Speed={speed}, Steering={final_steering}, Target Ratio={target_ratio:.3f}")
                                    continue

                                # Last forward to stop
                                elif state == States.LAST_RUN:
                                    end_sector_point = [250, 35] if is_clockwise else [150, 35]
                                    add_marker_point_wrapper("End Sector Point", end_sector_point[0], end_sector_point[1], color=(0, 0, 255), radius=2, label="Stop P")
                                    target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                    steering, previous_wall_error, _ = compute_wall_follow_steering(track["polygon"], is_clockwise, previous_wall_error, target_ratio_override=target_ratio)
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, move forward")
                                    print(f"[{state.name}][MOVE] Sent final run command: Speed={speed}, Steering={steering}")

                                    if get_track_distance(end_sector_point[0], end_sector_point[1])[0]:
                                        time.sleep(0.001)
                                        continue
                                    esp.send_command("0, 0, 0, motor stop")
                                    print(f"[{state.name}][STOP] Sent absolute motor stop command at end point marker.")
                                    end_time = time.perf_counter()
                                    run_OC1 = False
                                    reset_OC = True
                                    break

                                time.sleep(0.001)
                elif run_OC2:
                        send_command_logged("OC2")
                        while run_OC2:
                                reply = esp_replyNprint()
                                if reply == "EOC2":
                                        stop_vehicle("OC2 stop requested by ESP32")
                                        run_OC2 = False
                                        break
                                
                                set_color_block_detection(red=True, green=True, magenta=True)
                                obstacle, parking, track = get_latest_data()
                                
                                # --- Option A Integration for standalone OC2 Monitoring Loop ---
                                if obstacle["color"] in ["RED", "GREEN"]:
                                        print(f"[DETECT][OC2] {obstacle['color']} pillar spotted! Center X: {obstacle['center_x']}, Y: {obstacle['center_y']} | Width: {obstacle['width']}, Height: {obstacle['height']}")
                                        
                                time.sleep(0.001)

                else:
                        recorded_time = end_time - start_time
                        reply = esp_replyNprint()
                        if reply == "ROC1":
                                reset_OC = True
                                run_OC1 = True
                                continue
                        elif reply == "ROC2":
                                reset_OC = True
                                run_OC2 = True
                                continue
                time.sleep(0.001)

except KeyboardInterrupt:
        print("\nShutting down software pipeline gracefully...")
        try:
                esp.send_command("0, 0, 0, emergency_stop")
        except Exception:
                pass
        esp.disconnect()
        stop_vision_system()
        time.sleep(0.5)
