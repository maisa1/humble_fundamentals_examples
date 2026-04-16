#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import time
import sys
import threading
import subprocess
import os


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
    def spawn_turtle(self, x, y, theta, name):
        """Spawn a turtle at given position"""
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Turtle spawned: {name}')
            return True
        else:
            self.get_logger().error(f'Failed to spawn {name}')
            return False


def run_turtle1():
    """Run the non-PID turtle using ros2 run"""
    cmd = ['ros2', 'run', 'turtle_controller', 'turtle1_polygon']
    subprocess.run(cmd)


def run_turtle2():
    """Run the PID turtle using ros2 run"""
    cmd = ['ros2', 'run', 'turtle_controller', 'turtle2_polygon']
    subprocess.run(cmd)


def main(args=None):
    # First initialize ROS2
    rclpy.init(args=args)
    
    # Spawn the second turtle
    spawner = TurtleSpawner()
    success = spawner.spawn_turtle(x=5.5, y=5.5, theta=0.0, name='turtle2')
    
    if not success:
        print("Failed to spawn turtle2, exiting...")
        rclpy.shutdown()
        return
    
    # Destroy spawner node
    spawner.destroy_node()
    rclpy.shutdown()
    
    # Give time for turtle2 to be fully initialized
    time.sleep(2.0)
    
    # Run both turtles using subprocess
    print("Starting turtle1 (Non-PID) and turtle2 (PID)...")
    thread1 = threading.Thread(target=run_turtle1)
    thread2 = threading.Thread(target=run_turtle2)
    
    # Start threads
    thread1.start()
    thread2.start()
    
    # Wait for both threads to complete
    thread1.join()
    thread2.join()
    
    print("Both turtles have completed drawing all polygons!")


if __name__ == '__main__':
    main()