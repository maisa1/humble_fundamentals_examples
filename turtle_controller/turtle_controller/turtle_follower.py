#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import SetPen, Spawn
import math
import colorsys


class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        
        # Parameters
        self.follow_distance = 1.5  # Distance to maintain from leader
        self.max_speed = 2.0
        self.kp_linear = 1.5
        self.kp_angular = 4.0
        
        # State variables
        self.leader_pose = None
        self.follower_pose = None
        self.leader_name = 'turtle1'
        self.follower_name = 'turtle2'
        
        # Color variables - for continuous color changing
        self.hue = 0.0
        self.color_change_rate = 0.01  # How fast colors change
        self.last_color_update_time = 0.0
        
        # Spawn the follower turtle first
        self.spawn_follower_turtle()
        
        # Create service clients
        self.set_pen_client = self.create_client(SetPen, f'/{self.follower_name}/set_pen')
        
        # Wait a bit for services to be available
        self.create_timer(0.5, self.initialize_pen)
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.follower_name}/cmd_vel', 10)
        
        self.leader_pose_sub = self.create_subscription(
            TurtlePose, f'/{self.leader_name}/pose', 
            self.leader_pose_callback, 10)
        
        self.follower_pose_sub = self.create_subscription(
            TurtlePose, f'/{self.follower_name}/pose', 
            self.follower_pose_callback, 10)
        
        # Timer for control loop - faster for smoother following
        self.timer = self.create_timer(0.03, self.control_loop)  # ~33 Hz
        
        # Timer for color changing - slower for visible transitions
        self.color_timer = self.create_timer(0.1, self.update_color)  # 10 Hz
        
        self.get_logger().info('Turtle Follower Node Started')
        self.get_logger().info('Follower turtle will leave a permanent colorful trail!')
        self.get_logger().info('Use teleop to move turtle1 and watch turtle2 follow')
    
    def initialize_pen(self):
        """Initialize pen with first color"""
        if self.set_pen_client.service_is_ready():
            # Set initial color to red
            self.set_pen_color(255, 0, 0, 3)  # Red, width=3
            self.get_logger().info('Pen initialized with red color')
    
    def spawn_follower_turtle(self):
        """Spawn the second turtle"""
        self.get_logger().info('Spawning follower turtle...')
        
        spawn_client = self.create_client(Spawn, '/spawn')
        
        # Wait for spawn service
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Spawn turtle2 at a different position than turtle1
        request = Spawn.Request()
        request.x = 5.0  # Different from default turtle1 position
        request.y = 5.0
        request.theta = 0.0
        request.name = self.follower_name
        
        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info(f'Successfully spawned: {future.result().name}')
        else:
            self.get_logger().error('Failed to spawn turtle')
    
    def leader_pose_callback(self, msg):
        self.leader_pose = msg
    
    def follower_pose_callback(self, msg):
        self.follower_pose = msg
    
    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def set_pen_color(self, r, g, b, width=3):
        """Set pen color for follower turtle"""
        if self.set_pen_client.service_is_ready():
            req = SetPen.Request()
            req.r = r
            req.g = g
            req.b = b
            req.width = width
            req.off = 0  # Pen down (drawing mode)
            
            future = self.set_pen_client.call_async(req)
            return True
        return False
    
    def update_color(self):
        """Update the pen color continuously in rainbow pattern"""
        # Cycle through hue values (0 to 1)
        self.hue = (self.hue + self.color_change_rate) % 1.0
        
        # Convert HSV to RGB (all values 0-255)
        # HSV: Hue (0-1), Saturation (0-1), Value (0-1)
        rgb = colorsys.hsv_to_rgb(self.hue, 1.0, 1.0)  # Full saturation and value
        
        r = int(rgb[0] * 255)
        g = int(rgb[1] * 255)
        b = int(rgb[2] * 255)
        
        # Set the new pen color
        self.set_pen_color(r, g, b, 3)
    
    def control_loop(self):
        """Main control loop for following behavior"""
        if self.leader_pose is None or self.follower_pose is None:
            return
        
        # Calculate desired position (maintain follow_distance behind leader)
        leader_x = self.leader_pose.x
        leader_y = self.leader_pose.y
        leader_theta = self.leader_pose.theta
        
        # Calculate target position behind the leader
        target_x = leader_x - self.follow_distance * math.cos(leader_theta)
        target_y = leader_y - self.follow_distance * math.sin(leader_theta)
        
        # Calculate errors
        current_x = self.follower_pose.x
        current_y = self.follower_pose.y
        
        # Distance to target
        distance_error = self.calculate_distance(
            current_x, current_y, target_x, target_y)
        
        # Angle to target
        angle_to_target = math.atan2(
            target_y - current_y, target_x - current_x)
        current_angle = self.follower_pose.theta
        angle_error = self.normalize_angle(angle_to_target - current_angle)
        
        # Create control command
        cmd = Twist()
        
        # Linear velocity (proportional to distance error with saturation)
        cmd.linear.x = min(self.kp_linear * distance_error, self.max_speed)
        
        # If we're very close, slow down for smoother stopping
        if distance_error < 0.2:
            cmd.linear.x = 0.3 * distance_error
        
        # Angular velocity (proportional to angle error)
        cmd.angular.z = self.kp_angular * angle_error
        
        # Limit angular speed for smoother turns
        max_angular = 3.0
        if abs(cmd.angular.z) > max_angular:
            cmd.angular.z = max_angular if cmd.angular.z > 0 else -max_angular
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Optional: Log some info occasionally
        # if random.random() < 0.01:  # 1% chance
        #     self.get_logger().info(f'Distance error: {distance_error:.2f}, Angle error: {angle_error:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TurtleFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Optional: Lift pen when shutting down
        node.get_logger().info('Shutting down turtle follower...')
        # Note: To lift pen, we would set pen.off = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()