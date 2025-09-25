#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data  # keep this for MAVROS sensor QoS

class GpsSub(Node):
    def __init__(self):
        super().__init__('gps_sub')
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.cb,
            qos_profile_sensor_data
        )
        '''
        (delete later)
        qos_profile_sensor_data:
        - Send packet fast as possible
        - Don't retransmit dropped packets
        - Only see newest data
        '''

    def cb(self, msg):
        print(f"{msg.latitude:.7f}, {msg.longitude:.7f}, {msg.altitude:.2f}")
        # Insert code to input gps into robodog

def main():
    rclpy.init()
    node = GpsSub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

