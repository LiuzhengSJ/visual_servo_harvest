#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleVisualServo(Node):
    """A simple visual servoing node"""
    
    def __init__(self):
        super().__init__('simple_visual_servo')
        
        # Parameters
        self.declare_parameter('gain', 0.5)
        self.declare_parameter('max_velocity', 0.1)
        
        self.gain = self.get_parameter('gain').value
        self.max_vel = self.get_parameter('max_velocity').value
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Simulated error
        self.target_error = [0.2, 0.1, 0.0]
        
        self.get_logger().info('Simple Visual Servo Node Started')
        
    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = min(self.gain * self.target_error[0], self.max_vel)
        cmd.linear.y = min(self.gain * self.target_error[1], self.max_vel)
        cmd.linear.z = min(self.gain * self.target_error[2], self.max_vel)
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'vx={cmd.linear.x:.3f}, vy={cmd.linear.y:.3f}')
        
        # Simulate decreasing error
        self.target_error[0] *= 0.95
        self.target_error[1] *= 0.95

def main(args=None):
    rclpy.init(args=args)
    node = SimpleVisualServo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
