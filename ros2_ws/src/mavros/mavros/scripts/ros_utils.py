#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

def get_mavros_gps(
    topic: str = '/mavros/global_position/global',
    timeout_sec: float = 5.0
):
    """
    Wait for one NavSatFix from MAVROS and return (lat, lon, alt), or None on timeout.
    """

    class _GpsOnce(Node):
        def __init__(self):
            super().__init__('gps_once_helper')
            self.future: Future = Future()
            self.create_subscription(NavSatFix, topic, self._cb, qos_profile_sensor_data)

        def _cb(self, msg: NavSatFix):
            if not self.future.done():
                self.future.set_result((msg.latitude, msg.longitude, msg.altitude))

    if not rclpy.ok():
        rclpy.init()

    node = _GpsOnce()
    try:
        rclpy.spin_until_future_complete(node, node.future, timeout_sec=timeout_sec)
        return node.future.result() if node.future.done() else None
    finally:
        node.destroy_node()
        rclpy.shutdown()

