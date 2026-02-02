
#!/usr/bin/env python3
import math
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_close_turtle_first", True)

        self.catch_close_turtle_first_ = self.get_parameter(
            "catch_close_turtle_first").value

        self.turtle_to_catch_ : Turtle = None



         # PID constants
        self.kp_linear = 1.7
        self.kp_angular = 2.0       

        self.pose_ : Pose = None

         # Create subscribers and publishers
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist,"/turtle1/cmd_vel",10)
        
        self.pose_subscriber_ = self.create_subscription(
            Pose,"/turtle1/pose",self.pose_callback,10)
        
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray,"alive_turtles",self.callback_alive_turtles,10)
        
        self.call_catch_turtle_client_ = self.create_client(CatchTurtle,"catch_turtle",)
        
        # Control timer (runs at 10Hz)
        self.control_loop_timer_ = self.create_timer(0.7,self.control_loop)
       
        
    def pose_callback(self, pose:Pose):
        self.pose_ = pose

    def callback_alive_turtles(self , turtle_list: TurtleArray):
        if len(turtle_list.turtles) > 0 :
            if self.catch_close_turtle_first_ and self.pose_ is not None: 
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in turtle_list.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x*dist_x + dist_y * dist_y)
                    if closest_turtle is None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle

            else:
                self.turtle_to_catch_ = turtle_list.turtles[0]
           
    def control_loop(self):
        if self.pose_ is None or self.turtle_to_catch_ is None:
            return
        # Calculate distance to target
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x*dist_x + dist_y * dist_y)
        # Create velocity command
        cmd = Twist()
        # If we're close enough to target, stop
        if distance > 0.5:
            cmd.linear.x = self.kp_linear*distance
            goal_theta = math.atan2(dist_y,dist_x)
            diff = goal_theta - self.pose_.theta

            # Normalize angle difference to [-pi, pi]
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi :
                diff += 2*math.pi
            cmd.angular.z = self.kp_angular*diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_publisher_.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        while not self.call_catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for catch turtle service....")
        
        request = CatchTurtle.Request()
        request.name = turtle_name
        future = self.call_catch_turtle_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, turtle_name=turtle_name))
    
    def callback_call_catch_turtle_service(self, future,turtle_name):
        response : CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error("Turtle" + turtle_name + "could not be removed")
    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    node.get_logger().info("ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()