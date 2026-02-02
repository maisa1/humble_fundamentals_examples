
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.msg import Pose

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.publisher_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.subscriber_ = self.create_subscription(Pose ,"/turtle1/pose",self.subscriber_pose_callback,10)
        self.client_ = self.create_client(SetPen,"/turtle1/set_pen")

        # INITIALIZE THE VARIABLES HERE
        self.linear_vel = 2.8   # Default linear velocity
        self.angular_vel = 1.0  # Default angular velocity
        self.call_set_pen(255, 0, 225, 10, False)
        self.create_timer(0.1, self.data_callback)

    def call_set_pen(self,r,g,b,width,off):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for server")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_change_color)

    def callback_change_color(self , future):
        response = future.result()

    def data_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher_.publish(msg)


    def subscriber_pose_callback(self,message:Pose):
      # Store current position for logging
        current_x = message.x
        current_y = message.y
        
      # Define boundaries
        x_max = 10.0
        x_min = 2.0
        y_max = 10.0
        y_min = 2.0
        
        # Check if turtle is near the edges (outside the safe zone)
        if (current_x > x_max or current_x < x_min or 
            current_y > y_max or current_y < y_min):
            # Slow down near edges
            self.linear_vel = 1.5
            self.angular_vel = 0.8
            self.get_logger().warn(
                f"Near edge! Slowing down. Position: x={current_x:.2f}, y={current_y:.2f}"
            )
        else:
            # Normal speed in center area
            self.linear_vel = 2.8
            self.angular_vel = 1.0
        
        # Log position (throttled)
        self.get_logger().info(
            f"Turtle at: x={current_x:.2f}, y={current_y:.2f}, theta={message.theta:.2f}, "
            f"Speed: linear={self.linear_vel}, angular={self.angular_vel}",
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    node.get_logger().info("ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()

    
""" 
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.publisher_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.client_ = self.create_client(SetPen,"/turtle1/set_pen")
        self.call_set_pen(255, 0, 225, 10, False)
        self.create_timer(0.5, self.data_callback)


    def call_set_pen(self,r,g,b,width,off):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for server")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_change_color)

    def callback_change_color(self , future):
        response = future.result()

        
    def data_callback(self):
        msg = Twist()
        msg.linear.x = 2.5
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

       
def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    node.get_logger().info("ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()
 """
