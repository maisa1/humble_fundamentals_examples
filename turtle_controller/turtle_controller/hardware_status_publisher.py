
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces import String
from std_msgs.msg import String
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPubNode(Node):

    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.publisher_ = self.create_publisher(HardwareStatus,"hardware_status",10)
        self.timer_ =self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("hw status publisher has been started")
    
    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperture = 33.31
        msg.are_motors_ready = True
        msg.debug_message = "nothing"
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPubNode()
    node.get_logger().info("ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()