#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu

class SensorSubscriber(Node):
    
    battery_message = BatteryState()
    IMU_message = Imu()
    
    def __init__(self):
        super().__init__("Battery sensor messages")
        
        self.battery_subscriber = self.create_subscription(
            BatteryState, # message type
            "/mavros/battery", # topic
            self.callback,
            10
            )
        
        # self.IMU_subscriber = self.create_subscription(
        #     Imu, # message type
        #     "/mavros/imu/data", # topic
        #     self.callback,
        #     10
        #     )
        
        self.battery_subscriber
        # self.IMU_subscriber
        self.get_logger().info("starting subscriber node")
        
    def callback(self, battery_message): # for each subscriber need callback method
        # battery_message = BatteryState()
        # IMU_message = Imu()
        self.get_logger().info(f"Battery state: {battery_message}")
        
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