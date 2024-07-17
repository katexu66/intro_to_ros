#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

# computer IP: 169.254.68.200

class BlueRov2Sensors(Node):
    
    def __init__(self):
        super().__init__("bluerov2_sensors")
        
        qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )
        
        self.battery_message = None
        self.battery_subscriber = self.create_subscription(
            BatteryState, # message type
            "/mavros/battery", # topic
            self.battery_callback,
            qos_profile
            )
        
        self.IMU_message = None
        self.IMU_subscriber = self.create_subscription(
            Imu, # message type
            "/mavros/imu/data", # topic
            self.IMU_callback,
            qos_profile
            )
        
        self.battery_param = BatteryState()
        
        self.get_logger().info("starting subscriber node")
        
    def battery_callback(self, msg): # for each subscriber need callback method
        self.battery_param = msg
        self.get_logger().info(f"Voltage: {msg.voltage}, current: {msg.current}")
        
    def IMU_callback(self, msg):
        pass
        
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