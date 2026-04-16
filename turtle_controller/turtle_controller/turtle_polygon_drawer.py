#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class NonPIDPolygonDrawer(Node):
    def __init__(self, turtle_name='turtle1'):
        super().__init__(f'non_pid_polygon_drawer_{turtle_name}')
        
        self.turtle_name = turtle_name
        
        # Multiple polygons to draw (same as PID version)
        self.sides_list = [3, 4, 5, 6, 8]
        self.current_polygon_index = 0
        self.side_length = 2.0
        
        # Get current sides from the list
        self.sides = self.sides_list[self.current_polygon_index]
        
        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, f'/{turtle_name}/pose', self.pose_callback, 10)
        
        # Service client for setting pen
        self.pen_client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # States
        self.current_pose = None
        self.start_pose = None
        self.state = "WAIT"
        self.sides_drawn = 0
        self.pen_set = False
        
        # Tolerances
        self.distance_tolerance = 0.01
        self.angle_tolerance = 0.01
        
        # Control gains (Non-PID - simple proportional)
        self.k_linear = 1.0
        self.k_angular = 2.0
        
        # Speed limits
        self.max_linear_speed = 1.5
        self.max_angular_speed = 2.0
        
        self.get_logger().info(f"{self.turtle_name} - Non-PID Polygon Drawer Started: sides={self.sides}, length={self.side_length}")

    def pose_callback(self, pose: Pose):
        self.current_pose = pose

    def set_pen_according_to_sides(self):
        """Set pen color and width based on number of sides"""
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{self.turtle_name}: Service set_pen not available")
            return
            
        request = SetPen.Request()
        request.width = 6
        
        if self.sides == 3:
            request.r = 255
            request.g = 0
            request.b = 0
        elif self.sides == 4:
            request.r = 0
            request.g = 255
            request.b = 0
        elif self.sides == 5:
            request.r = 0
            request.g = 0
            request.b = 255
        elif self.sides == 6:
            request.r = 255
            request.g = 225
            request.b = 0
        elif self.sides == 8:
            request.r = 250  # Red 250, Green 125, Blue 10
            request.g = 125
            request.b = 10
        else:
            request.r = 255
            request.g = 255
            request.b = 255
            
        request.off = 0
        
        future = self.pen_client.call_async(request)
        future.add_done_callback(self.pen_set_callback)

    def pen_set_callback(self, future):
        """Callback for pen set service"""
        try:
            future.result()
            self.get_logger().info(f"{self.turtle_name}: Pen set for {self.sides}-sided polygon")
            self.pen_set = True
        except Exception as e:
            self.get_logger().error(f"{self.turtle_name}: Failed to set pen: {e}")

    def control_loop(self):
        if self.current_pose is None:
            return

        twist = Twist()

        if self.state == "WAIT":
            self.start_pose = self.current_pose
            if not self.pen_set:
                self.set_pen_according_to_sides()
                return  # Wait for pen to be set
            self.state = "FORWARD"
            self.get_logger().info(f"{self.turtle_name}: Starting first side of {self.sides}-gon...")

        elif self.state == "FORWARD":
            # Calculate remaining distance
            dist = math.sqrt((self.current_pose.x - self.start_pose.x) ** 2 +
                             (self.current_pose.y - self.start_pose.y) ** 2)
            error = self.side_length - dist

            if error <= self.distance_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.start_pose = self.current_pose
                self.state = "TURN"
                self.get_logger().info(f"{self.turtle_name}: Side {self.sides_drawn + 1} completed. Turning...")
            else:
                # Proportional control for linear speed
                speed = self.k_linear * error
                twist.linear.x = min(speed, self.max_linear_speed)
                twist.angular.z = 0.0  # No angular movement while going forward

        elif self.state == "TURN":
            target_angle = (2 * math.pi) / self.sides
            angle_turned = self.normalize_angle(self.current_pose.theta - self.start_pose.theta)
            error = target_angle - angle_turned

            if abs(error) <= self.angle_tolerance:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.sides_drawn += 1

                if self.sides_drawn >= self.sides:
                    # Move to next polygon
                    self.current_polygon_index += 1
                    if self.current_polygon_index < len(self.sides_list):
                        self.sides = self.sides_list[self.current_polygon_index]
                        self.sides_drawn = 0
                        self.pen_set = False
                        self.state = "WAIT"
                        self.get_logger().info(f"{self.turtle_name}: Starting {self.sides}-gon...")
                    else:
                        self.get_logger().info(f"{self.turtle_name}: All polygons completed!")
                        self.timer.cancel()
                        return
                else:
                    self.start_pose = self.current_pose
                    self.state = "FORWARD"
                    self.get_logger().info(f"{self.turtle_name}: Starting side {self.sides_drawn + 1} of {self.sides}-gon...")
            else:
                # Proportional control for angular speed
                speed = self.k_angular * error
                twist.angular.z = min(speed, self.max_angular_speed)
                twist.linear.x = 0.0  # No linear movement while turning

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        """Normalize angle to [0, 2π]"""
        while angle < 0:
            angle += 2 * math.pi
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        return angle


class PIDPolygonDrawer(Node):
    def __init__(self, turtle_name='turtle2'):
        super().__init__(f'pid_polygon_drawer_{turtle_name}')
        
        self.turtle_name = turtle_name
        self.sides_list = [3, 4, 5, 6, 8]
        self.current_polygon_index = 0
        self.side_length = 2.0
        
        self.sides = self.sides_list[self.current_polygon_index]
        
        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, f'/{turtle_name}/pose', self.pose_callback, 10)
        
        # Service client for setting pen
        self.pen_client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        
        # Wait for pen service
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.turtle_name}: Waiting for set_pen service...')
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # States
        self.current_pose = None
        self.start_pose = None
        self.target_angle = 0.0
        self.state = "WAIT_FOR_POSE"
        self.sides_drawn = 0
        self.pen_set = False
        
        # Tolerances
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.05
        self.alignment_tolerance = 0.1
        
        # PID gains for orientation
        self.kp = 2.5   # Proportional gain
        self.ki = 0.05  # Integral gain
        self.kd = 0.2   # Derivative gain
        
        # PID variables
        self.integral_error = 0.0
        self.prev_error = 0.0
        
        # Control gains for linear movement
        self.k_linear = 1.0
        self.k_angular = 2.0
        
        # Speed limits
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.5
        
        self.get_logger().info(f"{self.turtle_name} - PID Polygon Drawer Started: sides={self.sides}, length={self.side_length}")
        self.get_logger().info(f"PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def pose_callback(self, pose: Pose):
        if self.current_pose is None:
            self.get_logger().info(f"{self.turtle_name}: Received first pose at x={pose.x:.2f}, y={pose.y:.2f}")
        self.current_pose = pose

    def set_pen_according_to_sides(self):
        request = SetPen.Request()
        request.width = 6
        request.off = 0
        
        if self.sides == 3:
            request.r = 255
            request.g = 0
            request.b = 0
        elif self.sides == 4:
            request.r = 0
            request.g = 255
            request.b = 0
        elif self.sides == 5:
            request.r = 0
            request.g = 0
            request.b = 255
        elif self.sides == 6:
            request.r = 255
            request.g = 225
            request.b = 0
        elif self.sides == 8:
            request.r = 250  # Red 250, Green 125, Blue 10
            request.g = 125
            request.b = 10
        else:
            request.r = 255
            request.g = 255
            request.b = 255
            
        future = self.pen_client.call_async(request)
        future.add_done_callback(self.pen_set_callback)

    def pen_set_callback(self, future):
        try:
            future.result()
            self.get_logger().info(f"{self.turtle_name}: Pen set for {self.sides}-sided polygon")
            self.pen_set = True
        except Exception as e:
            self.get_logger().error(f"{self.turtle_name}: Failed to set pen: {e}")

    def calculate_target_angle(self):
        base_angle = (2 * math.pi / self.sides) * self.sides_drawn
        return base_angle

    def pid_control(self, error, dt=0.05):
        """Full PID controller"""
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral_error += error * dt
        self.integral_error = max(-10.0, min(10.0, self.integral_error))
        i_term = self.ki * self.integral_error
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        return p_term + i_term + d_term

    def control_loop(self):
        if self.current_pose is None:
            return

        twist = Twist()
        
        if self.state == "WAIT_FOR_POSE":
            if self.current_pose is not None:
                self.start_pose = self.current_pose
                if not self.pen_set:
                    self.set_pen_according_to_sides()
                else:
                    self.state = "FORWARD"
                    self.target_angle = self.calculate_target_angle()
                    self.integral_error = 0.0
                    self.prev_error = 0.0
                    self.get_logger().info(f"{self.turtle_name}: Starting side {self.sides_drawn + 1} of {self.sides}-gon...")
            return

        elif self.state == "FORWARD":
            dist = math.sqrt((self.current_pose.x - self.start_pose.x) ** 2 +
                             (self.current_pose.y - self.start_pose.y) ** 2)
            distance_error = self.side_length - dist

            current_angle = self.current_pose.theta
            desired_angle = self.target_angle
            angle_error = self.normalize_angle(desired_angle - current_angle)
            
            # PID control for orientation
            if abs(angle_error) > self.alignment_tolerance and dist > 0.1:
                angular_speed = self.pid_control(angle_error)
                twist.angular.z = min(max(angular_speed, -self.max_angular_speed), self.max_angular_speed)
                twist.linear.x = min(self.k_linear * distance_error, self.max_linear_speed) * 0.5
            else:
                if distance_error <= self.distance_tolerance:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.start_pose = self.current_pose
                    self.state = "TURN"
                    self.get_logger().info(f"{self.turtle_name}: Side {self.sides_drawn + 1} completed. Turning...")
                    self.integral_error = 0.0
                    self.prev_error = 0.0
                else:
                    speed = self.k_linear * distance_error
                    twist.linear.x = min(speed, self.max_linear_speed)
                    angular_speed = self.pid_control(angle_error) * 0.5
                    twist.angular.z = min(max(angular_speed, -self.max_angular_speed), self.max_angular_speed)

        elif self.state == "TURN":
            angle_turned = self.normalize_angle(self.current_pose.theta - self.start_pose.theta)
            target_turn = (2 * math.pi) / self.sides
            error = target_turn - angle_turned

            if abs(error) <= self.angle_tolerance:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.sides_drawn += 1

                if self.sides_drawn >= self.sides:
                    self.current_polygon_index += 1
                    if self.current_polygon_index < len(self.sides_list):
                        self.sides = self.sides_list[self.current_polygon_index]
                        self.sides_drawn = 0
                        self.pen_set = False
                        self.state = "WAIT_FOR_POSE"
                        self.get_logger().info(f"{self.turtle_name}: Starting {self.sides}-gon...")
                    else:
                        self.get_logger().info(f"{self.turtle_name}: All polygons completed!")
                        self.timer.cancel()
                        return
                else:
                    self.start_pose = self.current_pose
                    self.state = "FORWARD"
                    self.target_angle = self.calculate_target_angle()
                    self.integral_error = 0.0
                    self.prev_error = 0.0
            else:
                angular_speed = self.pid_control(error)
                twist.angular.z = min(max(angular_speed, -self.max_angular_speed), self.max_angular_speed)

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    # Create both nodes
    non_pid_node = NonPIDPolygonDrawer('turtle1')
    pid_node = PIDPolygonDrawer('turtle2')
    
    # Create executor and add both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(non_pid_node)
    executor.add_node(pid_node)
    
    # Spin both nodes together
    try:
        executor.spin()
    finally:
        executor.shutdown()
        non_pid_node.destroy_node()
        pid_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()