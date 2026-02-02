#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SpiralTurtle(Node):
    def __init__(self):
        super().__init__('spiral_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Spiral parameters
        self.linear_speed = 2.0  # Initial forward speed
        self.angular_speed = 1.0  # Initial turning speed
        self.speed_factor = 0.99  # Reduce speed by 1% each time
        
        # Create timer to publish commands
        self.timer = self.create_timer(0.1, self.move_spiral)  # 10 Hz
        self.counter = 0
        self.is_moving = True  # Flag to track if still moving
    
    def move_spiral(self):
        if not self.is_moving:
            return  # Stop publishing if we reached target
        
        # Calculate current linear speed
        current_linear = self.linear_speed * (self.speed_factor ** self.counter)
        
        # Check if we reached target speed (0.40)
        if current_linear <= 0.40:
            self.get_logger().info(f'Target reached! Linear: {current_linear:.2f}, Angular: {self.angular_speed:.2f}')
            
            # Send STOP command (all zeros)
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher_.publish(stop_msg)
            
            # Stop the timer and future publications
            self.is_moving = False
            self.timer.cancel()
            return
        
        # Continue with spiral movement
        msg = Twist()
        msg.linear.x = current_linear
        msg.angular.z = self.angular_speed  # Constant turn

        # OPTION 2: Increasing angular speed (alternative)
        # msg.linear.x = self.linear_speed  # Constant forward
        # msg.angular.z = self.angular_speed * (1.01 ** self.counter)  # Increasing turn
        
        self.publisher_.publish(msg)
        
        # Log the current speeds
        self.get_logger().info(f'Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SpiralTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()