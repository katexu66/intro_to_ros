#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("Battery sensor messages")
        
        self.subscriber = self.create_subscription(
            BatteryState, # message type
            "/mavros/battery", # topic
            self.my_callback,
            10
            )
        
        self.subscriber = self.create_subscription(
            Imu, # message type
            "/mavros/imu/data", # topic
            self.my_callback,
            10
            )
        
        self.subscriber
        self.get_logger().info("starting subscriber node")
        
    def callback(self, msg): # for each subscriber need callback method
        self.get_logger().info(f"Battery state: {BatteryState}, IMU: {Imu}")
        
def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()