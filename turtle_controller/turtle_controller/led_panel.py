
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedPanelState

class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel_state")
        self.declare_parameter("led_states", [0, 0, 0])  # Default: all LEDs off
        self.led_state_ = self.get_parameter("led_states").value

        self.publisher_ = self.create_publisher(LedPanelState,"led_panel_state",10)
        self.server_ = self.create_service(SetLed,"set_led", self.set_led_callback)
        self.get_logger().info("LED Panel Node Started")
        
    def publish_led_state(self):
        msg = LedPanelState()
        msg.led_states = self.led_state_
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published LED state: {self.led_state_}")


    def set_led_callback(self, request: SetLed.Request ,response: SetLed.Response):
        self.get_logger().info(f"Service called: LED {request.led_number} -> {'ON' if request.state else 'OFF'}")
        
        # 1. Validate the LED number (0, 1, or 2 for 3 LEDs)
        if request.led_number < 0 or request.led_number >= len(self.led_state_):
            response.success = False
            response.message = f"Invalid LED number {request.led_number}. Must be 0-{len(self.led_state_)-1}"
            self.get_logger().warn(response.message)
            return response
        
        # 2. Update the LED state
        # Remember: 0 = OFF, 1 = ON
        if request.state:  # If state is True (ON)
            self.led_state_[request.led_number] = 1
        else:  # If state is False (OFF)
            self.led_state_[request.led_number] = 0
        
        # 3. Set the response
        response.success = True
        response.message = f"LED {request.led_number} set to {'ON' if request.state else 'OFF'}"
        
        # 4. Publish the updated state
        self.publish_led_state()
      
        return response
       
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()