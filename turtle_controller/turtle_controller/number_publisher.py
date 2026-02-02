
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class PubNumNode(Node):

    def __init__(self):
        super().__init__("number_publisher")

        self.declare_parameter("data" ,2)
        self.declare_parameter("timer_period",0.5)
        
        self.publisher_ = self.create_publisher(Int64,"number",10)
        self.counter_ = 0
        self.get_logger().info("number node started")
        self.timer_period_ = self.get_parameter("timer_period").value
        self.create_timer(self.timer_period_, self.data_callback)
        
    def data_callback(self):
        msg = Int64()
        msg.data = self.get_parameter("data").value 
        self.get_logger().info(f": {msg.data}")  
        self.publisher_.publish(msg)
        self.counter_ +=1
       
def main(args=None):
    rclpy.init(args=args)
    node = PubNumNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()