#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from turtlesim.srv import Spawn, Kill
import time

class TimedTurtleSpawner(Node):
    def __init__(self):
        super().__init__('timed_turtle_spawner')
        
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        
        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        
        self.turtle_counter = 0
        self.spawn_time = {}  # Track spawn time for each turtle
        
        # Create timers
        self.spawn_timer = self.create_timer(1.0, self.spawn_turtle)
        self.removal_timer = self.create_timer(0.1, self.check_for_removals)  # Check every 100ms
        
        self.get_logger().info('Timed Turtle Spawner started!')
    
    def spawn_turtle(self):
        """Spawn a new turtle"""
        self.turtle_counter += 1
        turtle_name = f"temp_turtle_{self.turtle_counter}"
        
        # Random position
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 6.28)
        
        # Create spawn request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name
        
        # Call service asynchronously
        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda future, tname=turtle_name: self.spawn_done_callback(future, tname))
    
    def spawn_done_callback(self, future, turtle_name):
        """Callback when spawn is complete"""
        try:
            response = future.result()
            self.get_logger().info(f'Spawned turtle: {turtle_name}')
            
            # Record spawn time
            self.spawn_time[turtle_name] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {e}')
    
    def check_for_removals(self):
        """Check which turtles need to be removed (after 2 seconds)"""
        current_time = time.time()
        turtles_to_remove = []
        
        # Find turtles that have been alive for more than 2 seconds
        for turtle_name, spawn_time in list(self.spawn_time.items()):
            if current_time - spawn_time >= 2.0:
                turtles_to_remove.append(turtle_name)
        
        # Remove the turtles
        for turtle_name in turtles_to_remove:
            self.remove_turtle(turtle_name)
    
    def remove_turtle(self, turtle_name):
        """Remove a specific turtle"""
        request = Kill.Request()
        request.name = turtle_name
        
        future = self.kill_client.call_async(request)
        future.add_done_callback(
            lambda future, tname=turtle_name: self.remove_done_callback(future, tname))
    
    def remove_done_callback(self, future, turtle_name):
        """Callback when removal is complete"""
        try:
            future.result()
            self.get_logger().info(f'Removed turtle: {turtle_name}')
            
            # Remove from tracking
            if turtle_name in self.spawn_time:
                del self.spawn_time[turtle_name]
                
        except Exception as e:
            self.get_logger().warn(f'Failed to remove turtle {turtle_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TimedTurtleSpawner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()