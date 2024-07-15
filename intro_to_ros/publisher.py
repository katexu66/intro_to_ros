#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import random

class TutorialPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher") # start a node named tutorial_publisher
        self.publisher = self.create_publisher(
            # create publisher that broadcasts Vector3 messages to topic named "/tutorial/vector3"
            Vector3,
            "/tutorial/vector3",
            10
        )
        self.publisher_timer = self.create_timer(
            # create timer that runs run_node method every second (1 Hz freq)
            1.0, self.run_node
        )
        self.get_logger().info("starting publisher node") # log node starting in terminal
        
    def run_node(self):
        # publish random Vector3 messages
        msg = Vector3()
        msg.x = random.uniform(-10.0, 10.0)
        msg.y = random.uniform(-10.0, 10.0)
        msg.z = random.uniform(-10.0, 10.0)
        self.publisher.publish(msg)
        self.get_logger().info(f"Vector3\n\tx: {msg.x}\ty: {msg.y}\tz: {msg.z}")
        
def main(args=None):
    rclpy.init(args=args) # starts ROS2 Python3 client
    node = TutorialPublisher() # makes instasnce of TutorialPublisher class

    try:
        rclpy.spin(node) # keeps node running until exception
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node() # destroys node at end of program lifespan
        if rclpy.ok():
            rclpy.shutdown() # closes ROS2 client if still active

if __name__=="__main__":
    main()