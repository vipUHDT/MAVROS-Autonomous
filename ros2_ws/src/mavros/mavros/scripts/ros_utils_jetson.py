#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Vector3
from tf_transformations import euler_from_quaternion
import subprocess
import time

def get_mavros_attitude(
    gps_topic: str = '/mavros/global_position/global',
    rel_alt_topic: str = '/mavros/global_position/rel_alt',
    imu_topic: str = '/mavros/imu/data',
    timeout_sec: float = 5.0,
    degrees: bool = False,
):
    """
    Wait for one NavSatFix from MAVROS and return lat, and lon or None on timeout
    Return relative altitude from ground lock
    Return roll, pitch, yaw
    Angles are in radians unless changed in param
    Uses sensor-data QoS to match MAVROS publisher
    """
    class _AttitudeOnce(Node):
        def __init__(self):
            super().__init__('mavros_state_once_helper')
            self.future: Future = Future()
            self._lat = None
            self._lon = None
            self._rel_alt = None
            self._rpy = None

            # Subscribers for each data source
            self.create_subscription(NavSatFix, gps_topic, self._gps_cb, qos_profile_sensor_data)
            self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)
            from std_msgs.msg import Float64
            self.create_subscription(Float64, rel_alt_topic, self._rel_alt_cb, qos_profile_sensor_data)

        def _gps_cb(self, msg: NavSatFix):
            self._lat = float(msg.latitude)
            self._lon = float(msg.longitude)
            self._maybe_finish()

        def _rel_alt_cb(self, msg):
            self._rel_alt = float(msg.data)
            self._maybe_finish()

        def _imu_cb(self, msg: Imu):
            q = msg.orientation
            self._rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self._maybe_finish()

        def _maybe_finish(self):
            if self.future.done():
                return
            if (self._lat is not None and
                self._lon is not None and
                self._rel_alt is not None and
                self._rpy is not None):
                roll, pitch, yaw = self._rpy
                if degrees:
                    import math
                    roll = math.degrees(roll)
                    pitch = math.degrees(pitch)
                    yaw = math.degrees(yaw)
                self.future.set_result((self._lat, self._lon, self._rel_alt, roll, pitch, yaw))        

    created_ctx = False
    if not rclpy.ok():
        rclpy.init()
        created_ctx = True

    node = _AttitudeOnce()
    try:
        rclpy.spin_until_future_complete(node, node.future, timeout_sec=timeout_sec)
        return node.future.result() if node.future.done() else None
    finally:
        node.destroy_node()
        if created_ctx:
            rclpy.shutdown()

def publish_custom_gps(lat: float, lon: float, alt: float,
                       topic: str = '/gps_targets'):
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
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        pub = node.create_publisher(Vector3, topic, qos)

        msg = Vector3()
        msg.x = float(lat)
        msg.y = float(lon)
        msg.z = float(alt)
        
        for _ in range(50):  # ~5s total
            print("waiting for subscriber...")
            if pub.get_subscription_count() > 0:
                print("Detected subscriber")
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        # publish a few times over 3s to avoid race with late subscribers
        '''
        for i in range(3):
            pub.publish(msg)
            print("Sent")
            rclpy.spin_once(node, timeout_sec=0.2)
        '''
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(1)
    finally:
        node.destroy_node()
        if created_ctx:
            rclpy.shutdown()

def publish_custom_gps_cli(lat: float, lon: float, alt: float,
                            topic: str = '/gps_targets'):
    msg = f"{{x: {float(lat)}, y: {float(lon)}, z: {float(alt)}}}"

    cmd = [
        'ros2', 'topic', 'pub',
        topic,
        'geometry_msgs/msg/Vector3',
        msg,
        '--once'
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(result.stdout.strip())
    except subprocess.CalledProcessError as e:
        print(f"ros2 topic pub failed: {e.stderr.strip()}")
