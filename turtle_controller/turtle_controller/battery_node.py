#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.get_logger().info("Battery Node Started")
        
        # Create service client to call LED panel
        self.client_ = self.create_client(SetLed, "set_led")
        
        # Wait for service to be available
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for LED panel service...")
        
        # Battery starts FULL (as per activity)
        self.battery_full = True
        self.get_logger().info("Battery is FULL - Starting simulation")
        
        # Start the simulation
        self.start_simulation()
    
    def start_simulation(self):
        """
        Start the battery simulation cycle
        Activity requirement:
        - Battery empty after 4 seconds
        - Battery full after 6 more seconds
        - Repeat indefinitely
        """
        if self.battery_full:
            # Battery is full, will drain in 4 seconds
            self.get_logger().info("Battery draining... (empty in 4 seconds)")
            self.create_timer(4.0, self.battery_empty_callback)
        else:
            # Battery is empty, will charge in 6 seconds
            self.get_logger().info("Battery charging... (full in 6 seconds)")
            self.create_timer(6.0, self.battery_full_callback)
    
    def battery_empty_callback(self):
        """
        Called when battery becomes empty (after 4 seconds)
        """
        self.battery_full = False
        self.get_logger().info("Battery is EMPTY! Turning ON LED")
        
        # Call service to turn ON LED 0 (you can choose any LED)
        self.call_set_led_service(0, True)
        
        # Start charging phase (6 seconds to full)
        self.start_simulation()
    
    def battery_full_callback(self):
        """
        Called when battery becomes full (after 6 seconds)
        """
        self.battery_full = True
        self.get_logger().info(" Battery is FULL! Turning OFF LED")
        
        # Call service to turn OFF LED 0
        self.call_set_led_service(0, False)
        
        # Start draining phase (4 seconds to empty)
        self.start_simulation()
    
    def call_set_led_service(self, led_number, state):
        """
        Helper function to call the SetLed service
        """
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state
        
        # Make the service call
        future = self.client_.call_async(request)
        future.add_done_callback(self.service_response_callback)
    
    def service_response_callback(self, future):
        """
        Handle the service response
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✓ Service response: {response.message}")
            else:
                self.get_logger().error(f"✗ Service failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()