
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SkeletonNode(Node):

    def __init__(self):
        super().__init__("py_skeleton_oop_timer")
        self.counter_ = 0
        self.get_logger().info("Let's Begin")
        self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info("OOP Version!" + str(self.counter_))
        self.counter_ +=1
       
def main(args=None):
    rclpy.init(args=args)
    node = SkeletonNode()
    node.get_logger().info("ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()