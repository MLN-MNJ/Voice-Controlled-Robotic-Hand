# teleop_node.py
# --- RUN THIS INSIDE YOUR WSL 2 TERMINAL ---

import cv2
import mediapipe as mp
import urllib.request
import numpy as np
import rclpy  # Import ROS 2 client library
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type

# --- 1. Find your Windows IP Address ---
# Open CMD on WINDOWS (not WSL) and type: ipconfig
WINDOWS_IP = "0.0.0.0"  # <--- !!! REPLACE THIS WITH YOUR IP !!!

STREAM_URL = f'http://{WINDOWS_IP}:5000/video_feed'

# --- 2. The Classifier Function (THE NEW "BRAIN") ---
# This function looks at the hand landmarks and returns a string
def classify_hand_pose(hand_landmarks):
    
    # Get the Y-coordinates for the fingertips and their base knuckles
    # In MediaPipe, lower Y-value means "higher" on the screen.
    
    # Landmark IDs:
    # 8 = Index Tip, 5 = Index MCP (knuckle)
    # 12 = Middle Tip, 9 = Middle MCP
    # 16 = Ring Tip, 13 = Ring MCP
    # 20 = Pinky Tip, 17 = Pinky MCP
    
    landmarks = hand_landmarks.landmark
    
    # Rule for "point"
    # Index finger is extended (tip is "higher" than knuckle)
    index_extended = landmarks[8].y < landmarks[5].y
    # Other three fingers are curled (tips are "lower" than knuckles)
    middle_curled = landmarks[12].y > landmarks[9].y
    ring_curled = landmarks[16].y > landmarks[13].y
    pinky_curled = landmarks[20].y > landmarks[17].y

    if index_extended and middle_curled and ring_curled and pinky_curled:
        return "point"

    # Rule for "fist"
    # All four fingers are curled
    index_curled = landmarks[8].y > landmarks[5].y
    
    if index_curled and middle_curled and ring_curled and pinky_curled:
        return "fist"

    # Default gesture if no match
    # You can change this to "open" or "flat" if you find the right word
    return "default" 

# --- 3. The ROS 2 Node ---
class HandTeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')
        
        # Create a publisher for the /dexhand_gesture topic
        self.publisher_ = self.create_publisher(String, '/dexhand_gesture', 10)
        
        # --- MediaPipe setup ---
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)

        # --- Stream setup ---
        self.get_logger().info(f"Connecting to stream at {STREAM_URL}...")
        self.stream = urllib.request.urlopen(STREAM_URL)
        self.bytes_buffer = b''
        
        # To avoid spamming, store the last sent command
        self.last_command = ""

    def run(self):
        while True:
            try:
                # --- Read from stream (same as hand_tracker.py) ---
                self.bytes_buffer += self.stream.read(1024)
                a = self.bytes_buffer.find(b'\xff\xd8')
                b = self.bytes_buffer.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = self.bytes_buffer[a:b+2]
                    self.bytes_buffer = self.bytes_buffer[b+2:]
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                    # --- Process with MediaPipe (same as hand_tracker.py) ---
                    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    results = self.hands.process(image_rgb)

                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            # Draw the hand
                            self.mp_drawing.draw_landmarks(
                                image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                            
                            # --- !!! THE BRIDGE !!! ---
                            # 1. Classify the pose
                            command_str = classify_hand_pose(hand_landmarks)
                            
                            # 2. Only publish if the command is NEW
                            if command_str != self.last_command:
                                self.get_logger().info(f"New pose: {command_str}, publishing...")
                                
                                # 3. Create a ROS String message and publish it
                                msg = String()
                                msg.data = command_str
                                self.publisher_.publish(msg)
                                
                                # 4. Remember the last command
                                self.last_command = command_str

                    # Show the image (optional, but good for debugging)
                    cv2.imshow('WSL 2 Teleop Node', image)
                    if cv2.waitKey(5) & 0xFF == ord('q'):
                        break
                        
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                pass

        # Cleanup
        cv2.destroyAllWindows()
        self.hands.close()
        self.get_logger().info("Shutting down node.")

# --- 4. Main function to run the node ---
def main(args=None):
    rclpy.init(args=args)
    
    node = HandTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()