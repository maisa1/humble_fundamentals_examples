
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddToIntServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(AddTwoInts,"add_two_ints", self.add_two_ints_callback)
        self.get_logger().info("Add Two ints has been started!")

    def add_two_ints_callback(self, request: AddTwoInts.Request ,response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + "+" +
                               str(request.b) + "=" + str(response.sum))
        return response
        


def main(args=None):
    rclpy.init(args=args)
    node = AddToIntServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()