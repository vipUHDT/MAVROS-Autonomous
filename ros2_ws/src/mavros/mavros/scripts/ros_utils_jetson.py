#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Vector3

def get_mavros_gps(topic: str = '/mavros/global_position/global', timeout_sec: float = 5.0):
    """
    Wait for one NavSatFix from MAVROS and return (lat, lon, alt), or None on timeout.
    Uses sensor-data QoS to match MAVROS publisher.
    """
    class _GpsOnce(Node):
        def __init__(self):
            super().__init__('gps_once_helper')
            self.future: Future = Future()
            self.create_subscription(NavSatFix, topic, self._cb, qos_profile_sensor_data)

        def _cb(self, msg: NavSatFix):
            if not self.future.done():
                self.future.set_result((msg.latitude, msg.longitude, msg.altitude))

    created_ctx = False
    if not rclpy.ok():
        rclpy.init()
        created_ctx = True

    node = _GpsOnce()
    try:
        rclpy.spin_until_future_complete(node, node.future, timeout_sec=timeout_sec)
        return node.future.result() if node.future.done() else None
    finally:
        node.destroy_node()
        if created_ctx:
            rclpy.shutdown()

def publish_custom_gps(lat: float, lon: float,
                       topic: str = 'gps_targets'):
    """
    Publish one NavSatFix with given coordinates on `topic`.
    Uses QoS RELIABLE + TRANSIENT_LOCAL so late subscribers still see the last command.
    """
    created_ctx = False
    if not rclpy.ok():
        rclpy.init()
        created_ctx = True

    node = Node('custom_gps_publisher')
    try:
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        pub = node.create_publisher(Vector3, topic, qos)

        msg = Vector3()
        #msg.header.frame_id = 'map'
        msg.x = float(lat)
        msg.y = float(lon)
        #msg. = float(alt)

        # publish a few times over 3s to avoid race with late subscribers
        for _ in range(6):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if created_ctx:
            rclpy.shutdown()


