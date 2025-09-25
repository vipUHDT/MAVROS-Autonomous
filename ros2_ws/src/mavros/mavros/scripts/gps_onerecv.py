#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

TOPIC = '/mavros/global_position/global'

class GpsOnce(Node):
    def __init__(self):
        super().__init__('gps_once')
        self.sub = self.create_subscription(
                NavSatFix, 
                '/mavros/global_position/global', 
                self.cb, qos_profile_sensor_data
                )

        self.timer = self.create_timer(10.0, self.on_timeout)  # 10s timeout

    def cb(self, msg: NavSatFix):
        print(f"{msg.latitude:.7f}, {msg.longitude:.7f}, {msg.altitude:.2f}")
        rclpy.shutdown()

    def on_timeout(self):
        self.get_logger().error(f'Timeout waiting for {TOPIC}')
        rclpy.shutdown()
        sys.exit(1)

def main():
    rclpy.init()
    node = GpsOnce()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

