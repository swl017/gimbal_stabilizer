#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation

class GimbalStabilizer(Node):
    def __init__(self):
        super().__init__('gimbal_stabilizer')
        
        # Set up tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Set up QoS profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher for joint commands with reliable QoS
        self.command_publisher = self.create_publisher(
            JointState, 
            'isaac_joint_commands', 
            reliable_qos
        )

        self.current_gimbal_rpy_rad_publisher = self.create_publisher(
            Vector3, 
            'gimbal_state_rpy_rad', 
            reliable_qos
        )
        
        # Create subscribers with sensor data QoS
        self.joint_state_sub = self.create_subscription(
            JointState,
            'isaac_joint_states',
            self.joint_state_callback,
            sensor_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'state/pose',
            self.pose_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'mavros/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        self.gimbal_command_rpy_sub = self.create_subscription(
            Vector3,
            'gimbal_command_rpy_deg',
            self.gimbal_command_callback,
            sensor_qos
        )

        # Initialize state variables
        self.current_joint_states = None
        self.vehicle_roll = 0.0
        self.vehicle_pitch = 0.0
        self.vehicle_yaw = 0.0
        self.vehicle_quat = None
        self.gimbal_command_rpy_deg = None
        
        # Joint names for the gimbal
        self.joint_names = [
            'cgo3_vertical_arm_joint',    # Controls yaw
            'cgo3_horizontal_arm_joint',  # Controls roll
            'cgo3_camera_joint'           # Controls pitch
        ]
        
        # Create timer for periodic publishing (100Hz)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def joint_state_callback(self, msg):
        """Store the current joint states"""
        self.current_joint_states = msg
        current_roll = 0.0
        current_pitch = 0.0
        current_yaw = 0.0
        for joint in msg.name:
            if joint == 'cgo3_vertical_arm_joint':
                yaw_idx = msg.name.index(joint)
                current_yaw = msg.position[yaw_idx] + np.pi/2
            if joint == 'cgo3_horizontal_arm_joint':
                roll_idx = msg.name.index(joint)
                current_roll = msg.position[roll_idx]
            if joint == 'cgo3_camera_joint':
                pitch_idx = msg.name.index(joint)
                current_pitch = msg.position[pitch_idx]
        gimbal_rpy_rad = Vector3()
        gimbal_rpy_rad.x = current_roll
        gimbal_rpy_rad.y = current_pitch
        gimbal_rpy_rad.z = current_yaw
        self.current_gimbal_rpy_rad_publisher.publish(gimbal_rpy_rad)
        
    def imu_callback(self, msg):
        """Extract roll, pitch, and yaw from IMU data"""
        # Convert quaternion to euler angles using tf2
        euler = self.quaternion_to_euler(msg.orientation)
        self.vehicle_roll = euler[0]
        self.vehicle_pitch = euler[1]
        self.vehicle_yaw = euler[2]
        self.vehicle_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def quaternion_to_euler(self, quaternion: Quaternion) -> tuple:
        """Convert quaternion to euler angles using tf2"""
        # Calculate roll (x-axis rotation)
        sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Calculate pitch (y-axis rotation)
        sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Calculate yaw (z-axis rotation)
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def pose_callback(self, msg):
        """Extract roll and pitch from vehicle pose"""
        # Convert quaternion to euler angles using tf2
        euler = self.quaternion_to_euler(msg.pose.orientation)
        self.vehicle_roll = euler[0]
        self.vehicle_pitch = euler[1]
        self.vehicle_yaw = euler[2]

    def gimbal_command_callback(self, msg):
        self.gimbal_command_rpy_deg = msg

    def compute_stabilizing_commands(self):
        """Compute joint commands to counteract vehicle rotation"""
        # For perfect stabilization, we want to counter the vehicle rotation
        # Note: The signs might need to be inverted depending on joint conventions      
        rollpitch_in_horizontal_body_frame = (Rotation.from_euler('z', -self.vehicle_yaw) * Rotation.from_quat(self.vehicle_quat)).as_euler('zxy', degrees=False)
        if self.gimbal_command_rpy_deg is not None:
            rollpitch_in_horizontal_body_frame = (Rotation.from_euler('z', -self.gimbal_command_rpy_deg.z * np.pi/180.0) * Rotation.from_euler('z', -self.vehicle_yaw) * Rotation.from_quat(self.vehicle_quat)).as_euler('zxy', degrees=False)
        # stabilizing_roll = -self.vehicle_roll
        # stabilizing_pitch = -self.vehicle_pitch
        stabilizing_roll = -rollpitch_in_horizontal_body_frame[1]
        stabilizing_pitch = -rollpitch_in_horizontal_body_frame[2]
        stabilizing_yaw = -np.pi/2

        if self.gimbal_command_rpy_deg is not None:
            stabilizing_roll += (self.gimbal_command_rpy_deg.x * np.pi/180.0)
            stabilizing_pitch += (self.gimbal_command_rpy_deg.y * np.pi/180.0)
            stabilizing_yaw += (self.gimbal_command_rpy_deg.z * np.pi/180.0)

        return [stabilizing_yaw, stabilizing_roll, stabilizing_pitch]

    def timer_callback(self):
        """Publish stabilizing joint commands periodically"""
        # Compute stabilizing commands
        if self.current_joint_states is None or self.vehicle_quat is None:
            return
        joint_positions = self.compute_stabilizing_commands()
        
        # Create and populate message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_positions
        
        # Optional: Add velocity commands for better stabilization
        # msg.velocity = [0.0, 0.0, 0.0]
        
        # Publish commands
        self.command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    gimbal_stabilizer = GimbalStabilizer()
    
    try:
        rclpy.spin(gimbal_stabilizer)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        gimbal_stabilizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()