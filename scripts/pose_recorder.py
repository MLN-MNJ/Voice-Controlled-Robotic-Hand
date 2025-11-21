# pose_recorder.py
# RUN THIS IN YOUR WSL 2 TERMINAL

import cv2
import mediapipe as mp
import urllib.request
import numpy as np

# --- 1. Find your Windows IP Address ---
WINDOWS_IP = "0.0.0.0"  # <--- !!! REPLACE THIS WITH YOUR IP (no spaces!) !!!

STREAM_URL = f'http://{WINDOWS_IP}:5000/video_feed'

# --- 2. Helper Functions for Vector Math ---

def get_3d_vector(p1, p2):
    """Calculates a 3D vector from two MediaPipe landmarks."""
    return np.array([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])

def angle_between(v1, v2):
    """Calculates the angle in degrees between two 3D vectors."""
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    # Clip the dot product to be in [-1, 1] to avoid arccos errors
    dot_product = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.degrees(np.arccos(dot_product))

def calculate_curl_percentage(hand_landmarks):
    """
    Calculates the curl percentage (0.0 - 1.0) for 5 fingers.
    And the thumb yaw in degrees.
    """
    lm = hand_landmarks.landmark
    
    # --- 4 Main Fingers Curl ---
    # We define "curl" as the sum of the angles at the PIP and DIP joints.
    # A straight finger has angles ~0. A fully curled finger ~180.
    # We normalize this to a 0.0 - 1.0 percentage.
    MAX_CURL_ANGLE = 180  # 90 degrees at PIP + 90 at DIP

    # Index Finger
    v_index_pip = get_3d_vector(lm[6], lm[5]) # Bone 1 (MCP to PIP)
    v_index_dip = get_3d_vector(lm[7], lm[6]) # Bone 2 (PIP to DIP)
    v_index_tip = get_3d_vector(lm[8], lm[7]) # Bone 3 (DIP to Tip)
    index_pip_angle = angle_between(v_index_pip, v_index_dip)
    index_dip_angle = angle_between(v_index_dip, v_index_tip)
    index_curl = np.clip((index_pip_angle + index_dip_angle) / MAX_CURL_ANGLE, 0.0, 1.0)

    # Middle Finger
    v_middle_pip = get_3d_vector(lm[10], lm[9])
    v_middle_dip = get_3d_vector(lm[11], lm[10])
    v_middle_tip = get_3d_vector(lm[12], lm[11])
    middle_pip_angle = angle_between(v_middle_pip, v_middle_dip)
    middle_dip_angle = angle_between(v_middle_dip, v_middle_tip)
    middle_curl = np.clip((middle_pip_angle + middle_dip_angle) / MAX_CURL_ANGLE, 0.0, 1.0)

    # Ring Finger
    v_ring_pip = get_3d_vector(lm[14], lm[13])
    v_ring_dip = get_3d_vector(lm[15], lm[14])
    v_ring_tip = get_3d_vector(lm[16], lm[15])
    ring_pip_angle = angle_between(v_ring_pip, v_ring_dip)
    ring_dip_angle = angle_between(v_ring_dip, v_ring_tip)
    ring_curl = np.clip((ring_pip_angle + ring_dip_angle) / MAX_CURL_ANGLE, 0.0, 1.0)

    # Pinky Finger
    v_pinky_pip = get_3d_vector(lm[18], lm[17])
    v_pinky_dip = get_3d_vector(lm[19], lm[18])
    v_pinky_tip = get_3d_vector(lm[20], lm[19])
    pinky_pip_angle = angle_between(v_pinky_pip, v_pinky_dip)
    pinky_dip_angle = angle_between(v_pinky_dip, v_pinky_tip)
    pinky_curl = np.clip((pinky_pip_angle + pinky_dip_angle) / MAX_CURL_ANGLE, 0.0, 1.0)

    # --- Thumb Curl ---
    # We use a simpler 0-90 degree scale for the thumb
    v_thumb_mcp = get_3d_vector(lm[2], lm[1])
    v_thumb_ip  = get_3d_vector(lm[3], lm[2])
    v_thumb_tip = get_3d_vector(lm[4], lm[3])
    thumb_ip_angle = angle_between(v_thumb_ip, v_thumb_tip)
    thumb_curl = np.clip(thumb_ip_angle / 90.0, 0.0, 1.0)

    # --- Thumb Yaw ---
    # We'll define "yaw" as the angle between the thumb's
    # base vector and the hand's "index" vector.
    v_hand_plane = get_3d_vector(lm[5], lm[0]) # Vector from wrist to index
    v_thumb_plane = get_3d_vector(lm[1], lm[0]) # Vector from wrist to thumb base
    thumb_yaw = angle_between(v_hand_plane, v_thumb_plane)

    return {
        "index": index_curl,
        "middle": middle_curl,
        "ring": ring_curl,
        "pinky": pinky_curl,
        "thumb": thumb_curl,
        "thumb_yaw": thumb_yaw,
    }

def generate_code_for_pose(pose_data, letter):
    """
    Prints the Python function code for a given pose and letter.
    *** This version INVERTS the curl values to match the controller. ***
    """
    pose_name = f"set_pose_{letter}"
    
    # --- INVERT THE VALUES ---
    # Our translator calculates 1.0 = curled, but the controller
    # expects 0.0 = curled. We must "flip" them by (1.0 - value).
    
    p = pose_data # shorthand
    
    # We flip the 4 main fingers and the thumb curl
    inv_index  = 1.0 - p['index']
    inv_middle = 1.0 - p['middle']
    inv_ring   = 1.0 - p['ring']
    inv_pinky  = 1.0 - p['pinky']
    inv_thumb  = 1.0 - p['thumb']
    
    # We do NOT flip the yaw. It's an angle, not a percentage.
    thumb_yaw  = p['thumb_yaw']
    
    
    # --- Start building the Python code string ---
    code = f"""
    # ASL '{letter.upper()}' Pose
    def {pose_name}(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', {inv_index:.2f})
        self.set_finger_extension('middle', {inv_middle:.2f})
        self.set_finger_extension('ring', {inv_ring:.2f})
        self.set_finger_extension('pinky', {inv_pinky:.2f})
        self.set_finger_extension('thumb', {inv_thumb:.2f})

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', {thumb_yaw:.2f})
"""
    # --- End of code string ---

    print("--- GENERATED CODE (Copy and paste this into gesture_controller.py) ---")
    print(code)
    print("----------------------------------------------------------------------")

# --- 3. Main Loop ---
def run_recorder():
    # --- MediaPipe setup ---
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5)

    # --- Stream setup ---
    print(f"Connecting to stream at {STREAM_URL} ...")
    stream = urllib.request.urlopen(STREAM_URL)
    bytes_buffer = b''

    # To hold the last valid pose data
    last_pose_data = None

    while True:
        try:
            # --- Read from stream ---
            bytes_buffer += stream.read(1024)
            a = bytes_buffer.find(b'\xff\xd8')
            b = bytes_buffer.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes_buffer[a:b+2]
                bytes_buffer = bytes_buffer[b+2:]
                image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                # --- Process with MediaPipe ---
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # <-- Corrected function
                results = hands.process(image_rgb)

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Draw the hand
                        mp_drawing.draw_landmarks(
                            image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                        
                        # --- !!! THIS IS THE CORE !!! ---
                        # 1. Calculate the values
                        pose_data = calculate_curl_percentage(hand_landmarks)
                        last_pose_data = pose_data # Store the latest valid data

                        # 2. Display the values on the screen
                        y = 30
                        for finger, curl in pose_data.items():
                            text = f"{finger}: {curl:.2f}"
                            cv2.putText(image, text, (10, y), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            y += 30

                # Show the image
                cv2.imshow('WSL 2 Pose Recorder', image)

                # --- 4. The Code Generator ---
                key = cv2.waitKey(5) & 0xFF

                if key == ord('q'):
                    break
                
                # Check if the key is a letter (a-z)
                if key >= ord('a') and key <= ord('z'):
                    if last_pose_data:
                        letter = chr(key) # Convert the key code (e.g., 97) to a string ('a')
                        print(f"Captured pose for letter: {letter.upper()}")
                        # Call our new function with the last valid pose data
                        generate_code_for_pose(last_pose_data, letter)
                    else:
                        print("No hand detected. Cannot capture pose.")

        except Exception as e:
            print(f"Error: {e}")
            pass

    # Cleanup
    cv2.destroyAllWindows()
    hands.close()
    print("Stream closed.")

if __name__ == '__main__':
    run_recorder()