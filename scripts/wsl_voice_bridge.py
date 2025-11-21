# wsl_voice_bridge.py
# --- RUN THIS IN WSL 2 ---

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import time

# !!! USE YOUR WINDOWS IP HERE (Same as before) !!!
WINDOWS_IP = "0.0.0.0" 
SERVER_URL = f"http://{WINDOWS_IP}:5001/get_command"

class VoiceBridgeNode(Node):
    def __init__(self):
        super().__init__('voice_bridge')
        self.publisher_ = self.create_publisher(String, '/dexhand_gesture', 10)
        
        # Check for commands 5 times a second
        self.timer = self.create_timer(0.2, self.check_for_voice)
        self.get_logger().info(f"Connected to Voice Server at {SERVER_URL}")

    def check_for_voice(self):
        try:
            # Ask Windows: "Any new text?"
            response = requests.get(SERVER_URL, timeout=0.5)
            data = response.json()
            text = data.get('command', '')

            if text:
                self.get_logger().info(f"Received: '{text}'")
                self.spell_out_text(text)
                
        except Exception as e:
            self.get_logger().warn(f"Cannot reach Windows server: {e}")

    def spell_out_text(self, text):
        clean_text = text.lower()

        for char in clean_text:
            if 'a' <= char <= 'z':
                pose_command = f"pose_{char}"
                self.get_logger().info(f" -> Signing: {pose_command}")
                
                msg = String()
                msg.data = pose_command
                self.publisher_.publish(msg)
                time.sleep(1.5) 
            
            elif char == ' ':
                msg = String()
                msg.data = "reset"
                self.publisher_.publish(msg)
                time.sleep(1.0)

        msg = String()
        msg.data = "reset"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()