
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces import String
from std_msgs.msg import String

class publisherNode(Node):

    def __init__(self):
        super().__init__("publisher_test")
        self.publisher_ = self.create_publisher(String,"chatter",10)
        self.counter_ = 0
        self.get_logger().info("Publisher node started")
        self.create_timer(0.5, self.data_callback)
        
    def data_callback(self):
        msg = String()
        msg.data = f"Message #{self.counter_}: Hay! we did a publisher!"
        #msg.data = "Message #" + str(self.counter_) + ": Hay! we did a publisher!" #old way
        #msg.data = "Message #%d: Hay! we did a publisher!" % self.counter_  #older_formatting
        self.get_logger().info(f"Published: {msg.data}")  
        self.publisher_.publish(msg)
        self.counter_ +=1
       
def main(args=None):
    rclpy.init(args=args)
    node = publisherNode()
    node.get_logger().info("ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()