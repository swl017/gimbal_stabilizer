#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(JointState, '/px4_1/isaac_joint_commands', 10)
        
        # Create timer for periodic publishing (e.g., 100Hz)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Example joint names
        self.joint_names = ['cgo3_vertical_arm_joint', 'cgo3_horizontal_arm_joint', 'cgo3_camera_joint']

    def timer_callback(self):
        # Create message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Example joint positions (you can modify these values)
        msg.position = [0.0, 0.0, 0.0]
        
        # Optional: include velocities and efforts
        # msg.velocity = [0.0, 0.0, 0.0]
        # msg.effort = [-990.0, -990.0]
        
        # Publish the message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    joint_state_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        joint_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()