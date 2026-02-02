
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces import String
from std_msgs.msg import String

class subscriberNode(Node):

    def __init__(self):
        super().__init__("subscriber_test")
        self.subscriber_ = self.create_subscription(
            String,"chatter",self.subsciber_test_callback,10)
        self.counter_ = 0
        self.get_logger().info("subsciber node started")
        
    def subsciber_test_callback(self,msg:String):
        self.get_logger().info(msg.data)
        
       
def main(args=None):
    rclpy.init(args=args)
    node = subscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()