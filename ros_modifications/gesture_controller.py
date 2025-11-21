from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import threading
from std_msgs.msg import String

class GestureController(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('gesture_controller')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # Subscribe to gesture messages
        self.gesture_sub = self.create_subscription(String, 'dexhand_gesture', self.gesture_callback, 10)
        self.extension_sub = self.create_subscription(String, 'dexhand_finger_extension', self.extension_callback, 10)

        # Hardware publisher
        self.hw_pub = self.create_publisher(String, 'dexhand_hw_command', 10)

        # Run animation loop @ 30 Hz
        timer_period = 0.033 # seconds
        self.timer = self.create_timer(timer_period, self.animate_and_publish)

        # Initialize a base message which we will re-use for each frame
        self.NUM_JOINTS = 24
        self.joint_state = JointState()
        self.joint_state.name=[
                    'wrist_pitch_lower',
                    'wrist_pitch_upper',
                    'wrist_yaw',
                    'index_yaw',
                    'middle_yaw',
                    'ring_yaw',
                    'pinky_yaw',
                    'index_pitch',
                    'index_knuckle',
                    'index_tip',
                    'middle_pitch',
                    'middle_knuckle',
                    'middle_tip',
                    'ring_pitch',
                    'ring_knuckle',
                    'ring_tip',
                    'pinky_pitch',
                    'pinky_knuckle',
                    'pinky_tip',
                    'thumb_yaw',
                    'thumb_roll',
                    'thumb_pitch',
                    'thumb_knuckle',
                    'thumb_tip']

        self.joint_state.position = [0.0] * self.NUM_JOINTS
        self.get_logger().info("{0} publishing {1} joint states".format(self.nodeName, self.NUM_JOINTS))
        self.get_logger().info("{0} joint names: {1}".format(self.nodeName, self.joint_state.name))
        self.get_logger().info("{0} joint positions: {1}".format(self.nodeName, self.joint_state.position))

        # Utilities to make it easy to work with the fingers
        self.fingers = ['index', 'middle', 'ring', 'pinky', 'thumb']
        self.finger_joints = ['yaw', 'pitch', 'knuckle', 'tip']
        self.finger_joint_table = [
            ['index_yaw', 'index_pitch', 'index_knuckle', 'index_tip'],
            ['middle_yaw', 'middle_pitch', 'middle_knuckle', 'middle_tip'],
            ['ring_yaw', 'ring_pitch', 'ring_knuckle', 'ring_tip'],
            ['pinky_yaw', 'pinky_pitch', 'pinky_knuckle', 'pinky_tip'],
            ['thumb_yaw', 'thumb_pitch', 'thumb_knuckle', 'thumb_tip']
        ]
        self.finger_range_table = [
            [0.0, 75.0, 75.0, 75.0],    # index
            [0.0, 75.0, 75.0, 75.0],    # middle
            [0.0, 75.0, 75.0, 75.0],    # ring
            [0.0, 75.0, 75.0, 75.0],    # pinky
            [0.0, 60.0, 60.0, 60.0]]    # thumb


        # Intent lookup table - maps intent strings to functions
        self.intent_table = {
            "default": self.set_default_pose,
            "reset": self.set_default_pose,
            "wave": self.set_default_pose,
            "fist": self.set_fist_pose,
            "grab": self.set_grab_pose,
            "peace": self.set_peace_pose,
            "horns": self.set_horns_pose,
            "shaka": self.set_shaka_pose,
            "point": self.set_index_point_pose,
            "pose_a": self.set_pose_a,
            "pose_b": self.set_pose_b,
            "pose_c": self.set_pose_c,
            "pose_d": self.set_pose_d,
            "pose_e": self.set_pose_e,
            "pose_f": self.set_pose_f,
            "pose_g": self.set_pose_g,
            "pose_h": self.set_pose_h,
            "pose_i": self.set_pose_i,
            "pose_j": self.set_pose_j,
            "pose_k": self.set_pose_k,
            "pose_l": self.set_pose_l,
            "pose_m": self.set_pose_m,
            "pose_n": self.set_pose_n,
            "pose_o": self.set_pose_o,
            "pose_p": self.set_pose_p,
            "pose_q": self.set_pose_q,
            "pose_r": self.set_pose_r,
            "pose_s": self.set_pose_s,
            "pose_t": self.set_pose_t,
            "pose_u": self.set_pose_u,
            "pose_v": self.set_pose_v,
            "pose_w": self.set_pose_w,
            "pose_x": self.set_pose_x,
            "pose_y": self.set_pose_y,
            "pose_z": self.set_pose_z,
            "test": self.test_pose
        }

        # Really simple animation system to simulate servo motors moving
        # Basically, the target position, and a rotational velocity. 
        # Move at the velocity toward the target position
        self.joint_state_animation_targets = [0.0] * self.NUM_JOINTS
        self.joint_state_animation_velocities = [0.0] * self.NUM_JOINTS
        self.ROTATIONAL_VELOCITY = 0.05
        self.isAnimating = False
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

        except KeyboardInterrupt:
            pass

    # Sets the target position for a joint
    def set_joint_target_degrees(self, joint_name, target_position):
        if joint_name in self.joint_state.name:
            joint_index = self.joint_state.name.index(joint_name)
            self.joint_state_animation_targets[joint_index] = target_position * pi / 180.0

            # Set the velocity to move toward the target
            if (self.joint_state_animation_targets[joint_index] > self.joint_state.position[joint_index]):
                self.joint_state_animation_velocities[joint_index] = self.ROTATIONAL_VELOCITY
            else:
                self.joint_state_animation_velocities[joint_index] = -self.ROTATIONAL_VELOCITY
        else:
            self.get_logger().warn("{0} is not a valid joint name".format(joint_name))
    

    # Performs a single frame of animation and publishes the joint state
    def animate_and_publish(self):

        self.isAnimating = False
    
        # Animate all joints
        for i in range(self.NUM_JOINTS):
            # Calculate the difference between the target and current position
            diff = self.joint_state_animation_targets[i] - self.joint_state.position[i]
            # If the difference is greater than the rotational velocity, move the joint
            if abs(diff) > self.ROTATIONAL_VELOCITY:
                self.joint_state.position[i] += self.joint_state_animation_velocities[i]
                self.isAnimating = True # We have at least one joint moving
            # Otherwise, we're close enough to the target, so just set the position
            else:
                self.joint_state.position[i] = self.joint_state_animation_targets[i]
                self.joint_state_animation_velocities[i] = 0.0

        # Timestamp
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        
        # Publish the new joint state
        self.joint_pub.publish(self.joint_state)


    # Set default pose
    def set_default_pose(self):

        # All joints move to 0 degrees
        for joint_name in self.joint_state.name:
            self.set_joint_target_degrees(joint_name, 0.0)

        self.set_joint_target_degrees('index_yaw', -10.0)
        self.set_joint_target_degrees('middle_yaw', 0.0)
        self.set_joint_target_degrees('ring_yaw', 13.0)
        self.set_joint_target_degrees('pinky_yaw', 20.0)

    # Fist
    def set_fist_pose(self):
        self.set_default_pose()

        # Close the finger extensions
        for finger in self.fingers:
            self.set_finger_extension(finger, 0.0)

        # Rotate the yaw on the thumb to rotate the thumb around the palm
        self.set_finger_extension('thumb', 0.4)
        self.set_joint_target_degrees('thumb_yaw', 45.0)

    # ASL 'A' Pose
    def set_pose_a(self):
        # 1. Start from the default pose (just like 'fist' does)
        self.set_default_pose()

        # 2. Close all the finger extensions (just like 'fist' does)
        for finger in self.fingers:
            self.set_finger_extension(finger, 0.0)

        # 3. Set the thumb for the 'A' pose
        #    (curled, but with no yaw)
        self.set_finger_extension('thumb', 0.5)
        self.set_joint_target_degrees('thumb_yaw', 0.0)
        self.set_joint_target_degrees('thumb_roll', 15.0)
    

    # ASL 'B' Pose
    def set_pose_b(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)
        self.set_finger_extension('ring', 1.0)
        self.set_finger_extension('pinky', 1.0)
        self.set_finger_extension('thumb', 0.2)

        self.set_joint_target_degrees('index_yaw', -10.0)
        self.set_joint_target_degrees('middle_yaw', 0.0)
        self.set_joint_target_degrees('ring_yaw', 13.0)
        self.set_joint_target_degrees('pinky_yaw', 20.0)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 45)
        #self.set_joint_target_degrees('thumb_roll', -15.0)

    # ASL 'C' Pose
    def set_pose_c(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.50)
        self.set_finger_extension('middle', 0.50)
        self.set_finger_extension('ring', 0.50)
        self.set_finger_extension('pinky', 0.50)

        self.set_joint_target_degrees('index_yaw', -10.0)
        self.set_joint_target_degrees('middle_yaw', 0.0)
        self.set_joint_target_degrees('ring_yaw', 13.0)
        self.set_joint_target_degrees('pinky_yaw', 20.0)

        # 3. Set thumb yaw
        self.set_finger_extension('thumb', 0.60)                 # Curl
        self.set_joint_target_degrees('thumb_yaw', 49.60)        # Swing
        self.set_joint_target_degrees('thumb_roll', 45.0)
     # ASL 'D' Pose
    def set_pose_d(self):
        # 1. Start from the default pose
        self.set_default_pose()

        self.set_joint_target_degrees('index_yaw', -15.0)
        self.set_joint_target_degrees('middle_yaw', 0.0)
        self.set_joint_target_degrees('ring_yaw', 13.0)
        self.set_joint_target_degrees('pinky_yaw', 20.0)
        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.95)
        self.set_finger_extension('middle', 0.16)
        self.set_finger_extension('ring', 0.11)
        self.set_finger_extension('pinky', 0.07)
        self.set_finger_extension('thumb', 0.5)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 30.67)


    # ASL 'E' Pose
    def set_pose_e(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.1)
        self.set_finger_extension('middle', 0.1)
        self.set_finger_extension('ring', 0.1)
        self.set_finger_extension('pinky', 0.1)
        self.set_finger_extension('thumb', 0.3)    # Curls thumb *partially*
        self.set_joint_target_degrees('thumb_yaw', 35.0) 
        self.set_joint_target_degrees('thumb_roll', -23.0)

    # ASL 'F' Pose
    def set_pose_f(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.32)
        self.set_finger_extension('middle', 0.94)
        self.set_finger_extension('ring', 0.96)
        self.set_finger_extension('pinky', 0.96)
        self.set_finger_extension('thumb', 0.6)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 30.34)
        self.set_joint_target_degrees('thumb_roll', 13)


# ASL 'G' Pose
    def set_pose_g(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions
        self.set_finger_extension('index', 1.0)    # Straight
        self.set_finger_extension('middle', 0.0)   # Curled
        self.set_finger_extension('ring', 0.0)     # Curled
        self.set_finger_extension('pinky', 0.0)    # Curled
        self.set_finger_extension('thumb', 1.0)    # Straight

        # 3. Set finger yaw (index points straight)
        self.set_joint_target_degrees('index_yaw', 0.0)

        # 4. Set thumb yaw and roll (parallel to index)
        self.set_joint_target_degrees('thumb_yaw', 35.34)
        self.set_joint_target_degrees('thumb_roll', 13.0)

        
    # ASL 'H' Pose
    def set_pose_h(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)
        self.set_finger_extension('ring', 0.00)
        self.set_finger_extension('pinky', 0.00)
        self.set_finger_extension('thumb', 0.26)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 6.39)

    # ASL 'I' Pose
    def set_pose_i(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.0)
        self.set_finger_extension('middle', 0.0)
        self.set_finger_extension('ring', 0.0)
        self.set_finger_extension('pinky', 0.91)
        self.set_finger_extension('thumb', 0.49)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 20.14)

    # ASL 'J' Pose
    def set_pose_j(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.0)
        self.set_finger_extension('middle', 0.0)
        self.set_finger_extension('ring', 0.0)
        self.set_finger_extension('pinky', 0.91)
        self.set_finger_extension('thumb', 0.49)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 20.14)
    # ASL 'K' Pose
    def set_pose_k(self):
        # 1. Start from the default pose
        self.set_default_pose()

        self.set_joint_target_degrees('index_yaw', 5.0)
        self.set_joint_target_degrees('middle_yaw', -5.0)
        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.95)
        self.set_finger_extension('middle', 0.93)
        self.set_finger_extension('ring', 0.0)
        self.set_finger_extension('pinky', 0.0)
        self.set_finger_extension('thumb', 0.41)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 10.85)
        self.set_joint_target_degrees('thumb_roll', 10.85)



    # ASL 'L' Pose
    def set_pose_l(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 0.0)
        self.set_finger_extension('ring', 0.0)
        self.set_finger_extension('pinky', 0.0)
        self.set_finger_extension('thumb', 1.0)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', -29.33)


    # ASL 'M' Pose
    def set_pose_m(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.19)
        self.set_finger_extension('middle', 0.10)
        self.set_finger_extension('ring', 0.07)
        self.set_finger_extension('pinky', 0.36)
        self.set_finger_extension('thumb', 0.91)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 40.86)

    # ASL 'N' Pose
    def set_pose_n(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.16)
        self.set_finger_extension('middle', 0.09)
        self.set_finger_extension('ring', 0.17)
        self.set_finger_extension('pinky', 0.11)
        self.set_finger_extension('thumb', 0.97)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 34.65)


    # ASL 'O' Pose
    def set_pose_o(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.16)
        self.set_finger_extension('middle', 0.10)
        self.set_finger_extension('ring', 0.07)
        self.set_finger_extension('pinky', 0.11)
        self.set_finger_extension('thumb', 0.21)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 44.74)


    # ASL 'P' Pose
    def set_pose_p(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.95)
        self.set_finger_extension('middle', 0.14)
        self.set_finger_extension('ring', 0.21)
        self.set_finger_extension('pinky', 0.13)
        self.set_finger_extension('thumb', 0.83)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 31.30)

    # ASL 'Q' Pose
    def set_pose_q(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.97)
        self.set_finger_extension('middle', 0.15)
        self.set_finger_extension('ring', 0.11)
        self.set_finger_extension('pinky', 0.07)
        self.set_finger_extension('thumb', 0.95)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 29.04)

    # ASL 'R' Pose
    def set_pose_r(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.94)
        self.set_finger_extension('middle', 0.93)
        self.set_finger_extension('ring', 0.12)
        self.set_finger_extension('pinky', 0.27)
        self.set_finger_extension('thumb', 0.66)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 42.03)


    # ASL 'S' Pose
    def set_pose_s(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.0)
        self.set_finger_extension('middle', 0.00)
        self.set_finger_extension('ring', 0.00)
        self.set_finger_extension('pinky', 0.00)
        self.set_finger_extension('thumb', 0.57)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 29.38)

    # ASL 'T' Pose
    def set_pose_t(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.25)
        self.set_finger_extension('middle', 0.20)
        self.set_finger_extension('ring', 0.18)
        self.set_finger_extension('pinky', 0.12)
        self.set_finger_extension('thumb', 0.73)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 29.39)

    # ASL 'U' Pose
    def set_pose_u(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)
        self.set_finger_extension('ring', 0.0)
        self.set_finger_extension('pinky', 0.0)
        self.set_finger_extension('thumb', 0.20)
        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 31.96)


    # ASL 'V' Pose
    def set_pose_v(self):
        # 1. Start from the default pose
        self.set_default_pose()
        self.set_joint_target_degrees('index_yaw', 0.0)
        self.set_joint_target_degrees('middle_yaw', 0.0)
        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)
        self.set_finger_extension('ring', 0.0)
        self.set_finger_extension('pinky', 0.0)
        self.set_finger_extension('thumb', 0.20)
        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 31.96)

    # ASL 'W' Pose
    def set_pose_w(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)
        self.set_finger_extension('ring', 1.0)
        self.set_finger_extension('pinky', 0.0)
        self.set_finger_extension('thumb', 0.0)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 37.73)

    # ASL 'X' Pose
    def set_pose_x(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.77)
        self.set_finger_extension('middle', 0.17)
        self.set_finger_extension('ring', 0.24)
        self.set_finger_extension('pinky', 0.25)
        self.set_finger_extension('thumb', 0.69)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 35.12)

    # ASL 'Y' Pose
    def set_pose_y(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # Close all fingers except pinky and thumb
        for finger in self.fingers:
            if finger != 'pinky' and finger != 'thumb':
                self.set_finger_extension(finger, 0.0)

        # Angle pinky outward
        self.set_joint_target_degrees('pinky_yaw', -20.0)


    # ASL 'Z' Pose
    def set_pose_z(self):
        # 1. Start from the default pose
        self.set_default_pose()

        # 2. Set finger extensions (Using INVERTED values)
        self.set_finger_extension('index', 0.94)
        self.set_finger_extension('middle', 0.20)
        self.set_finger_extension('ring', 0.19)
        self.set_finger_extension('pinky', 0.15)
        self.set_finger_extension('thumb', 0.54)

        # 3. Set thumb yaw
        self.set_joint_target_degrees('thumb_yaw', 34.47)

    # Grab
    def set_grab_pose(self):
        self.set_default_pose()

        # Close the finger extensions partially
        for finger in self.fingers:
            self.set_finger_extension(finger, 0.45)

        # Rotate the yaw on the thumb to rotate the thumb around the palm
        self.set_joint_target_degrees('thumb_yaw', 40.0)

    # Peace
    def set_peace_pose(self):
        self.set_fist_pose()

        # Extend the index and middle fingers
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)

        # Spread the index and middle fingers
        self.set_joint_target_degrees('index_yaw', 15.0)
        self.set_joint_target_degrees('middle_yaw', -15.0)
    
    def test_pose(self):
        self.set_default_pose()

        self.set_finger_extension('thumb', 0.0)    # Curls thumb *partially*
        self.set_joint_target_degrees('thumb_yaw', 45.0) 
        self.set_joint_target_degrees('thumb_roll', -23.0)

    # Horns
    def set_horns_pose(self):
        self.set_fist_pose()

        # Extend the index and pinky fingers
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('pinky', 1.0)

    # Shaka
    def set_shaka_pose(self):
        self.set_default_pose()

        # Close all fingers except pinky and thumb
        for finger in self.fingers:
            if finger != 'pinky' and finger != 'thumb':
                self.set_finger_extension(finger, 0.0)

        # Angle pinky outward
        self.set_joint_target_degrees('pinky_yaw', -20.0)


    # Index finger point
    def set_index_point_pose(self):
        self.set_fist_pose()

        # Extend the index finger
        self.set_finger_extension('index', 1.0)

        

    # Sets the extension of a finger by name. 0.0 is closed (to fist) and 1.0 is open (straight)
    def set_finger_extension(self, finger_name, extension):
        if finger_name in self.fingers:
            finger_index = self.fingers.index(finger_name)

            # Clamp extension range from 0-1
            extension = min(max(extension, 0.0), 1.0)

            # Scale the range based on the extension value for each joint except yaw
            for i in range(1, len(self.finger_joints)):
                joint_name = self.finger_joint_table[finger_index][i]
                self.set_joint_target_degrees(joint_name, (1.0-extension) * self.finger_range_table[finger_index][i])
            
        else:
            self.get_logger().warn("{0} is not a valid finger name".format(finger_name))

    # Gesture message handler
    def gesture_callback(self, msg):
        self.get_logger().info('Received gesture: "%s"' % msg.data)

        # Look up the intent in the table and call the function
        if msg.data in self.intent_table:
            # Call sim function
            self.intent_table[msg.data]()

            # Publish the message to the hardware
            hw_msg = String()
            hw_msg.data = "gesture:{0}".format(msg.data)
            self.hw_pub.publish(hw_msg)

        else:
            self.get_logger().warn("{0} is not a known gesture".format(msg.data))

            
    # Finger extension message handler
    def extension_callback(self, msg):
        self.get_logger().info('Received finger extension: "%s"' % msg.data)

        # Parse the finger name and extension value from the message
        finger_name, extension = msg.data.split(':')
        extension = float(extension)

        # Set the finger extension in simulation
        if (finger_name in self.fingers):
            # Process sim finger
            self.set_finger_extension(finger_name, extension)

            # Hardware publisher operates in 0-100 range, so multiply by 100 and convert to int
            extension = int(extension * 100)

            # Hardware publisher user finger index and not name
            finger_index = self.fingers.index(finger_name)

            # Publish the message
            hw_msg = String()
            hw_msg.data = "fingerextension:{0}:{1}".format(finger_index, extension)
            self.hw_pub.publish(hw_msg)

        else:
            self.get_logger().warn("{0} is not a valid finger name".format(finger_name))
        
        
def main():
    node = GestureController()

if __name__ == '__main__':
    main()