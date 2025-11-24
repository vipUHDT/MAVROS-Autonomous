#!/usr/bin/env python3

import os
import time
import json
import threading
import subprocess
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ROS messages / services
from mavros_msgs.msg import State, WaypointReached, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterType

# Old local project imports 
#from Search_Area import plan_mission, export_map
#from Config import Config

import tf_transformations as tf  # for Euler

# Image processing imports
import cv2
import numpy as np

from image_processing.camera.controllers import *
from image_processing import PlatformState
from image_processing.camera import CameraMetadata
from image_processing.odcl.detection import *
from image_processing.odcl.Localize import *
from image_processing.odcl.detection import DetectionManager
from image_processing.data import DataManager,ImageDataset
from image_processing.tools import timestamp


class WaypointMission(Node):
    """
    ROS2 autonomous mission node:
      - MAVROS state + setpoints
      - Search-area mission
      - Hadron camera capture
      - ODCL detection + GPS target publishing
    """

    def __init__(self):
        super().__init__('auto_script')

        # --- State and synchronization ---
        self.current_state = State()
        self.current_waypoint_index = 0
        self.waypoint_reached_event = threading.Event()

        # Sensor placeholders
        self.imu_msg = None
        self.gps_msg = None
        self.rel_alt_msg = None
        self.sensor_event = threading.Event()

        # --- Search-area waypoints only ---
        self.search_area_waypoints = []

        # End mission hover point
        self.end_mission_latitude = 21.4001583
        self.end_mission_longitude = -157.7644202
        self.end_mission_altitude = 5

        # --- Timing arrays ---
        self.attitude_time = []
        self.search_area_waypoint_time = []
        self.individual_search_wp_time = []

        # !----- ROS2 Publishers / Subscribers -----!

        self.param_client = self.create_client(ParamSetV2, '/mavros/param/set')

        # Setpoint publisher (global raw setpoint)
        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget,
            '/mavros/setpoint_raw/global',
            10
        )

        # GPS target publisher (for Robodog or other consumers)
        gps_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.gps_target_pub = self.create_publisher(
            Vector3,
            '/gps_targets',
            gps_qos
        )

        # Vehicle state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Mission waypoint reached
        self.wp_reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.reached_callback,
            10
        )

        # Sensor topics
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile_sensor_data
        )

        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            qos_profile_sensor_data
        )

        # --- Filesystem setup ---
        #self.clean()  # remake images dir + clear old test file

        # --- Search-area waypoints setup (Config + Search_Area) ---
        #self.setup_search_area_waypoints()
        self.load_search_area_waypoints()

        # --- ODCL / camera stack init (from your test script) ---
        self.init_odcl_stack()

        self.get_logger().info("AUTONOMOUS SCRIPT IS READY!")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def state_callback(self, msg: State):
        self.current_state = msg

    def reached_callback(self, msg: WaypointReached):
        self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached")
        self.current_waypoint_index += 1
        self.waypoint_reached_event.set()

    def imu_callback(self, msg: Imu):
        self.imu_msg = msg
        self.sensor_event.set()

    def gps_callback(self, msg: NavSatFix):
        self.gps_msg = msg
        self.sensor_event.set()

    def rel_alt_callback(self, msg: Float64):
        self.rel_alt_msg = msg
        self.sensor_event.set()

    # ------------------------------------------------------------------
    # Utility / setup methods
    # ------------------------------------------------------------------

    def clean(self):
        """
        Remake directory for images and clear old ODCL waypoint file.
        """
        images_dir = "images"
        if os.path.exists(images_dir):
            import shutil
            shutil.rmtree(images_dir)
        os.mkdir(images_dir)

        test_file = "/home/uhdt/ros2_ws/ip_waypoints/test.txt"
        if os.path.exists(test_file):
            os.remove(test_file)

    def load_search_area_waypoints(self):
        """
        Load waypoints from JSON file
        """
        # Change this path if needed
        wp_path = "/home/uhdt/Documents/waypoints/waypoints.json"
        if not os.path.exists(wp_path):
            self.get_logger().warn(f"Search area waypoints file not found: {wp_path}")
            return

        with open(wp_path, 'r') as f:
            self.search_area_waypoints = json.load(f)["search_waypoints"]
        self.get_logger().info(f"Loaded {len(self.search_area_waypoints)} search waypoints")

    def set_param(self, name: str, value: float, force_set: bool = True) -> bool:
        """
        ROS2 version using /mavros/param/set (ParamSetV2).

        name  : ArduPilot parameter name "WPNAV_SPEED"
        value : numeric value (we send it as DOUBLE)
        """
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/mavros/param/set service not available')
            return False

        req = ParamSetV2.Request()
        req.force_set = force_set
        req.param_id = name

        # Fill rcl_interfaces/ParameterValue
        req.value.type = ParameterType.PARAMETER_DOUBLE   # == 3
        req.value.double_value = float(value)

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to set {name}: {e}')
            return False

        if resp.success:
            self.get_logger().info(f'Successfully set {name} = {value}')
        else:
            self.get_logger().warn(f'Param set failed for {name} (value={value})')

        return resp.success

    # ------------------------------------------------------------------
    # ODCL / Camera init (from your test script)
    # ------------------------------------------------------------------

    def init_odcl_stack(self):
        """
        Initialize Hadron camera, DataManager, and DetectionManager
        using your existing test configuration.
        """
        # Data file
        dataset_container = timestamp()
        img_folder_base = Path.cwd() / dataset_container
        data_filepath = Path.cwd() / f"{dataset_container}.hdf5"

        self.camera_metadata = CameraMetadata(6.51456, 4.897152, 1920, 1080, 28)
        self.georef_engine = Georeference_Engine("utm", 0)


        sahi_config = SahiConfig(
            slice=False,
            single_prediction=True
        )

        model_config = ModelConfig(
            backend="sahi",
            model_type="ultralytics",
            model_path="pipes.pt",
            confidence_threshold=0.7,
            device="cuda:0",   # change to "cpu" if no GPU
            backend_config=sahi_config
        )
        # Datasets
        datasets = [
            ImageDataset('rgb', (1080, 1920, 3), 'uint8'),
            ImageDataset('ir', (512, 640, 3), 'uint8'),
            ImageDataset('fusion', (720, 1080), 'uint8')
        ]

        for dataset in datasets:
            img_folder_container = img_folder_base / dataset.name 
            dataset.directory = img_folder_container
            img_folder_container.mkdir(parents=True, exist_ok=True)

        self.data_manager = DataManager(str(data_filepath), datasets)
        self.data_manager.initialize()

        self.detection_manager = DetectionManager(
            model_config,
            self.camera_metadata,
            self.georef_engine,
            self.sendGPS,
        )

        # Hadron camera setup
        self.camera = Hadron640R()
        self.camera.setConnection("192.168.0.1", "192.168.0.237", "root", "oelinux123")
        self.camera.initialize()
        #camera.setPort(5000)
        #camera.setTXPipeline()
        #camera.setRXPipeline()
        #camera.initialize()
        #camera.initializeStream([])
        #camera.startRXPipeline()
        time.sleep(5.0)

        self.get_logger().info("Hadron camera and ODCL initialized")


    def get_attitude(self, timeout_sec: float = 5.0, degrees: bool = False):
        """
        Wait for IMU + GPS + rel_alt data (if needed) and return:
            lat, lon, alt, roll, pitch, yaw

        Returns None on timeout.
        """
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.imu_msg is not None and self.gps_msg is not None and self.rel_alt_msg is not None:
                q = self.imu_msg.orientation
                roll, pitch, yaw = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])

                if degrees:
                    roll = math.degrees(roll)
                    pitch = math.degrees(pitch)
                    yaw = math.degrees(yaw)

                lat = float(self.gps_msg.latitude)
                lon = float(self.gps_msg.longitude)
                alt = float(self.rel_alt_msg.data)

                return lat, lon, alt, roll, pitch, yaw

            time.sleep(0.05)

        self.get_logger().warn("get_attitude(): timeout waiting for sensor data")
        return None

    def is_armed(self):
        """
        Block until the UAS is armed.
        """
        self.get_logger().info("Waiting to be ARMED...")
        while not self.current_state.armed:
            self.get_logger().info("Waiting to be ARMED...")
            time.sleep(1)
        self.get_logger().info("ARMED")
        time.sleep(0.5)

    def is_guided(self):
        """
        Block until the UAS mode is GUIDED.
        """
        self.get_logger().info("Waiting to be GUIDED...")
        while self.current_state.mode != "GUIDED":
            self.get_logger().info(f"Current mode: {self.current_state.mode}, waiting for GUIDED...")
            time.sleep(1)
        self.get_logger().info("GUIDED")
        self.get_logger().info("--------------------------- MISSION STARTING ---------------------------")

    def send_global_position_target(self, lat: float, lon: float, alt: float):
        """
        Compile a GlobalPositionTarget and publish it.
        """
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = 6           # FRAME_GLOBAL_INT
        msg.type_mask = 4088               # ignore everything except position
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)

        self.setpoint_pub.publish(msg)
        self.get_logger().info(f"Sent GlobalPositionTarget: lat={lat}, lon={lon}, alt={alt}")

    def publish_gps_target(self, lat: float, lon: float, alt: float = 0.0):
        """
        Publish a Vector3 on /gps_targets (lat, lon, alt).
        """
        msg = Vector3()
        msg.x = float(lat)
        msg.y = float(lon)
        msg.z = float(alt)
        self.gps_target_pub.publish(msg)
        self.get_logger().info(f"Published GPS target: lat={lat}, lon={lon}, alt={alt}")

    def publish_gps_target_cli(self, lat: float, lon: float, alt: float,
                               topic: str = '/gps_targets'):
        """
        Fire-and-forget wrapper around `ros2 topic pub` that publishes one Vector3
        on the given topic without blocking the caller.
        """
        msg = f"{{x: {float(lat)}, y: {float(lon)}, z: {float(alt)}}}"

        cmd = [
            'ros2', 'topic', 'pub',
            topic,
            'geometry_msgs/msg/Vector3',
            msg,
            '--once'
        ]

        def worker():
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, check=True)
                self.get_logger().info(result.stdout.strip())
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"ros2 topic pub failed: {e.stderr.strip()}")

        # Start the CLI publish in a background thread
        t = threading.Thread(target=worker, daemon=True)
        t.start()


    # ------------------------------------------------------------------
    # Mission logic: search-area + ODCL (no waypoint lap)
    # ------------------------------------------------------------------

    def search_area_command(self):
        """
        Fly through all search_area_waypoints sequentially.
        At each waypoint:
          - Wait until reached
          - Get attitude from MAVROS
          - Send attitude to IP
          - Capture Hadron RGB + IR camera
          - Store via DataManager
          - Run ODCL
          - If GPS target detected, publish on /gps_targets
        """

        if not self.search_area_waypoints:
            self.get_logger().warn("No search area waypoints loaded, aborting search_area_command.")
            return

        self.get_logger().info("-- Now conducting the search area --")
        start = time.time()

        iteration = 0

        for i, (lat_wp, lon_wp, alt_wp) in enumerate(self.search_area_waypoints):
            counter = 0
            startI = time.time()

            self.waypoint_reached_event.clear()

            self.get_logger().info(
                f"Search Waypoint {i}: lat={lat_wp}, lon={lon_wp}, alt={alt_wp}"
            )
            self.send_global_position_target(lat_wp, lon_wp, alt_wp)

            self.get_logger().info(f"Waiting for search waypoint {i} to be reached...")
            self.waypoint_reached_event.wait()
            self.get_logger().info(f"Reached search waypoint {i}")

            # Get attitude/GPS
            attitude = self.get_attitude(timeout_sec=10.0, degrees=False)
            if attitude:
                lat, lon, alt, roll, pitch, yaw = attitude

                self.get_logger().info(
                f"Attitude at WP {i}: lat={lat}, lon={lon}, alt={alt}, "
                f"roll={roll}, pitch={pitch}, yaw={yaw}"
            )
            else:
                self.get_logger("Error: no MAVROS messages received for attitude")
            

            # --- Capture from Hadron camera ---
            print("Cpaturing")


            image, ir_image = self.camera.capture()

            time.sleep(2)
            img_name = timestamp()

             # Store in DataManager
            if image is not None:
                print("Saving RGB image")
                #self.data_manager.append_frame("rgb", image)
                
                rgb_save_path= self.data_manager.image_datasets[self.data_manager.dataset_map['rgb']].directory / f"rgb-{img_name}.png"
                
                with open(str(self.data_manager.image_datasets[self.data_manager.dataset_map['rgb']].directory / f"{img_name}.txt"), 'w') as file:
                    file.write(f"{lat} {lon} {alt} {roll} {pitch} {yaw}")
                	

                cv2.imwrite(rgb_save_path, image)
            
            if ir_image is not None:
                print("Saving IR Image")
                #self.data_manager.append_frame("ir", ir_image)
                ir_save_path= self.data_manager.image_datasets[self.data_manager.dataset_map['ir']].directory / f"ir-{img_name}.png"
                cv2.imwrite(ir_save_path, ir_image)


            # ODCLls
            if isinstance(image, cv2.typing.MatLike):
                #cv2.imwrite(f"test_images/test-counter-{counter}.jpg", image)
                counter += 1
                queued_image = QueuedImage(image, PlatformState(alt, lat, lon, pitch, yaw, roll))
                self.detection_manager.queueImage(queued_image)
                self.detection_manager.processQueuedImages()
                self.detection_manager.update()
                #print(detection_manager.getGPS())
                
                
                # Take camera pic
                #img = np.zeros((640, 512, 3), dtype=np.uint8) #camera.captureFrame()
                #platform_state = PlatformState(alt, lat, lon, pitch, yaw, roll)
                #detection_manager.queueImage(QueuedImage(image, platform_state))
            
            #detection_manager.processQueue()
            #self.detection_manager.processQueuedImages()
            #self.detection_manager.update()

            time.sleep(3)
            self.get_logger().info(
                f"Iteration: {iteration}, "
                f"Active threads: {len(self.detection_manager.active_threads)}, "
                f"Results queue: {self.detection_manager.results_queue.qsize()}, "
                f"Image queue: {self.detection_manager.image_queue.qsize()}, "
                f"GPS queue: {self.detection_manager.gps_queue.qsize()}"
            )
            iteration += 1

            '''
            # Send Robodog
            gps_coords = self.detection_manager.getGPS()
            self.get_logger().info(f"Target GPS: {gps_coords}")

            if (gps_coords):
                target_lat, target_lon = gps_coords

                #self.publish_gps_target(target_lat, target_lon, 0.0, topic='/gps_targets')
                self.publish_gps_target_cli(target_lat, target_lon, 0.0, topic='/gps_targets')

                self.get_logger().info("!-------- Sent target to Robodog --------!")
            else:
                self.get_logger().info(f"Detected nothing at waypoint {i}")
            '''
  


            '''
            # --- Capture from Hadron camera ---
            image, ir_image = self.camera.capture()

            # Store in DataManager
            self.data_manager.append_frame("rgb", image)
            self.data_manager.append_frame("ir", ir_image)

            # ODCL
            if isinstance(image, cv2.typing.MatLike):
                #cv2.imwrite(f"test_images/test-counter-{counter}.jpg", image)
                counter += 1
                queued_image = QueuedImage(image, PlatformState(alt, lat, lon, pitch, yaw, roll))
                self.detection_manager.queueImage(queued_image)
                self.detection_manager.processQueuedImages()
                self.detection_manager.update()
                #print(detection_manager.getGPS())
                
                
                # Take camera pic
                #img = np.zeros((640, 512, 3), dtype=np.uint8) #camera.captureFrame()
                #platform_state = PlatformState(alt, lat, lon, pitch, yaw, roll)
                #detection_manager.queueImage(QueuedImage(image, platform_state))
            
            #detection_manager.processQueue()
            self.detection_manager.processQueuedImages()
            self.detection_manager.update()

            time.sleep(3)
            self.get_logger().info(
                f"Iteration: {iteration}, "
                f"Active threads: {len(self.detection_manager.active_threads)}, "
                f"Results queue: {self.detection_manager.results_queue.qsize()}, "
                f"Image queue: {self.detection_manager.image_queue.qsize()}, "
                f"GPS queue: {self.detection_manager.gps_queue.qsize()}"
            )
            iteration += 1

            # Send Robodog
            gps_coords = self.detection_manager.getGPS()
            self.get_logger().info(f"Target GPS: {gps_coords}")

            if (gps_coords):
                target_lat, target_lon = gps_coords

                #self.publish_gps_target(target_lat, target_lon, 0.0, topic='/gps_targets')
                self.publish_gps_target_cli(target_lat, target_lon, 0.0, topic='/gps_targets')

                self.get_logger().info("!-------- Sent target to Robodog --------!")
            else:
                self.get_logger().info(f"Detected nothing at waypoint {i}")
            '''

            time.sleep(1.0)
            elapsedI = time.time() - startI
            self.individual_search_wp_time.append(elapsedI)

        # LOGGING
        elapsed = time.time() - start
        self.search_area_waypoint_time.append(elapsed)
        self.get_logger().info("UAS COMPLETED SEARCHING THE AREA")

    def sendGPS(self, gps_coords):
            #gps_coords = self.detection_manager.getGPS()
            self.get_logger().info(f"Target GPS: {gps_coords}")

            if (gps_coords):
                target_lat, target_lon = gps_coords

                #self.publish_gps_target(target_lat, target_lon, 0.0, topic='/gps_targets')
                self.publish_gps_target_cli(target_lat, target_lon, 0.0, topic='/gps_targets')

                self.get_logger().info("!-------- Sent target to Robodog --------!")
            else:
                self.get_logger().info(f"Detected nothing at waypoint")

    def end_mission(self):
        """
        Fly to the predefined end-mission hover point.
        """
        self.waypoint_reached_event.clear()
        self.get_logger().info("Sending End Mission waypoint...")
        self.send_global_position_target(
            self.end_mission_latitude,
            self.end_mission_longitude,
            self.end_mission_altitude,
        )
        self.get_logger().info("Waiting for End Mission waypoint to be reached...")
        self.waypoint_reached_event.wait()
        self.get_logger().info("End Mission Waypoint Reached")
        self.get_logger().info(f"Search Area Time: {self.search_area_waypoint_time}")
        self.get_logger().info(f"Individual waypoint time: {self.individual_search_wp_time}")

    def run_full_mission(self):
        """
        Complete mission: wait for ARMED + GUIDED, then run search-area,
        then go to end-mission waypoint.
        """
        self.is_armed()
        self.is_guided()
        
        '''
        self.send_global_position_target(21.4002113, -157.7645811, 5)
        self.get_logger().info(f"Waiting for waypoint to be reached...")
        self.waypoint_reached_event.wait()
        self.get_logger().info(f"Reached search waypoint")
        time.sleep(2)
        
        self.waypoint_reached_event.clear()
        
        self.send_global_position_target(21.4001452, -157.7647454, 5)
        self.get_logger().info(f"Waiting for waypoint to be reached...")
        self.waypoint_reached_event.wait()
        self.get_logger().info(f"Reached search waypoint")
        time.sleep(2)
        '''
        self.search_area_command()

        #time.sleep(30)
        
        self.end_mission()

        self.get_logger().info("Mission Completed.")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        """
        Ensure DataManager and camera are properly shut down.
        """
        self.get_logger().info("Shutting down: closing DataManager and camera")
        try:
            if hasattr(self, "data_manager"):
                self.data_manager.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing DataManager: {e}")

        try:
            if hasattr(self, "camera"):
                self.camera.terminate()
        except Exception as e:
            self.get_logger().warn(f"Error terminating camera: {e}")

        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointMission()

    node.set_param('WPNAV_SPEED', 1000)      # cm/s
    node.set_param('WPNAV_SPEED_DN', 500)    # cm/s
    node.set_param('WPNAV_SPEED_UP', 500)    # cm/s
    node.set_param('GUID_OPTIONS', 64)       # DO NOT CHANGE

    mission_thread = threading.Thread(
        target=node.run_full_mission,
        daemon=True,
    )
    mission_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
