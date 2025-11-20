# CURRENTLY ONLY FOR STATIONARY TESTING
# NEEDS TO CHANGE THINGS FOR ACTUAL FLIGHT TESTING

from ros_utils_jetson import get_mavros_attitude, publish_custom_gps, publish_custom_gps_cli

from image_processing.camera.controllers import *
from image_processing import PlatformState
from image_processing.camera import CameraMetadata
from image_processing.odcl.detection import *
from image_processing.odcl.Localize import *
from image_processing.odcl.detection import DetectionManager
from image_processing.data import DataManager,ImageDataset
from image_processing.tools import timestamp
import os
import cv2
import time
import numpy as np
import random
from pathlib import Path

data_filepath = Path.cwd() / f"{timestamp()}.hdf5"
print("akflasjfkjasldjfl")
def random_platform_state(
    altitude_range=(0, 200),         
    latitude_range=(-90, 90),        
    longitude_range=(-180, 180),     
    pitch_range=(-180, 180),         
    yaw_range=(-180, 180),           
    roll_range=(-180, 180)           
) -> PlatformState:
    
    altitude  = random.uniform(*altitude_range)
    latitude  = random.uniform(*latitude_range)
    longitude = random.uniform(*longitude_range)
    pitch     = random.uniform(*pitch_range)
    yaw       = random.uniform(*yaw_range)
    roll      = random.uniform(*roll_range)

    return PlatformState(altitude, latitude, longitude, pitch, yaw, roll)

camera_metadata = CameraMetadata(6.51456, 4.897152, 1920, 1080, 28)
georeference_engine = Georeference_Engine("utm", 0)

detection_model = SahiDetectionModel(
    model_type="ultralytics",
    model_path="yolo11n.pt",
    confidence_threshold=0.7,
    device="cpu",
)

sahi_config = SahiConfig(
    slice=False, single_prediction=True
)

model_config = ModelConfig(
    backend = "sahi",
    model_type = "ultralytics",
    model_path = "yolo11n.pt",
    confidence_threshold = 0.7,
    device = "cuda:0",
    backend_config = sahi_config
)

datasets = [
    ImageDataset('rgb', (1080, 1920, 3), 'uint8'),
    ImageDataset('ir', (512, 640), 'uint8'),
    ImageDataset('fusion', (720, 1080), 'uint8')
]

data_manager = DataManager(data_filepath, datasets)
data_manager.initialize()

detection_manager = DetectionManager(model_config, camera_metadata, georeference_engine)
# Can delete later

#coords = get_mavros_gps(timeout_sec=10)
# Returns tuple (lat, lon, alt)
'''
coords = 21.296619, -157.815557, 0
if coords:
    lat, lon, alt = coords
    print(lat, lon)

    #publish_custom_gps(lat, lon, alt, topic='/gps_targets')
    publish_custom_gps_cli(lat, lon, alt, topic='/gps_targets')
else:
    print("No GPS fix in time")
'''

camera = Hadron640R()
camera.setConnection("192.168.0.1", "192.168.0.237", "root", "oelinux123")
camera.initialize()
#camera.setPort(5000)
#camera.setTXPipeline()
#camera.setRXPipeline()
#camera.initialize()
#camera.initializeStream([])
#camera.startRXPipeline()
time.sleep(5)



def main():
    counter = 0
    print("test")
    iteration = 0
    numberOfScans = 0
    gpsStoredLat = []
    gpsStoredLon = []
    seconds = 5
    lat = lon = alt = roll = pitch = yaw = None

    # init hadron and detectionManager
    
    print("Init IP successful")
    # Maybe also print camera settings for verifications


    print("Starting Search Area...")
    try:
        while True:
            
            #userInput = input("Press Enter to begin scan of area or q to exit: ")
            #if userInput == "q":
            #    print("Exiting")
            #    print(f"Scan ended with: {numberOfScans} scans")
            #    print(f"GPS of scanned area Lat: {gpsStoredLat}")
            #    print(f"GPS of scanned area Lat: {gpsStoredLon}")
            #    return 0
            
            # Countdown Timer 
            #for i in range(seconds, 0, -1):
            #    time.sleep(1)
            #    print(i)
            
            print("Starting scan")
            
            # Pull Drone GPS/Data
            attitude = get_mavros_attitude(timeout_sec=10)
            if attitude:
                lat, lon, alt, roll, pitch, yaw = attitude

            else:
                print("Error: no MAVROS messages received for attitude")
            print(lat, lon, alt, roll, pitch, yaw)

            image, ir_image = camera.capture()

            data_manager.append_frame("rgb", image)
            data_manager.append_frame("ir", ir_image)
            
            if isinstance(image, cv2.typing.MatLike):
                #cv2.imwrite(f"test_images/test-counter-{counter}.jpg", image)
                counter += 1
                queued_image = QueuedImage(image, PlatformState(alt, lat, lon, pitch, yaw, roll))
                detection_manager.queueImage(queued_image)
                detection_manager.processQueuedImages()
                detection_manager.update()
                #print(detection_manager.getGPS())
                
                
                # Take camera pic
                #img = np.zeros((640, 512, 3), dtype=np.uint8) #camera.captureFrame()
                #platform_state = PlatformState(alt, lat, lon, pitch, yaw, roll)
                #detection_manager.queueImage(QueuedImage(image, platform_state))

            # ODCL
            #detection_manager.processQueue()
            detection_manager.processQueuedImages()
            detection_manager.update()

            # Store stuff

            gps_coords = detection_manager.getGPS()
            print(gps_coords)
            if (gps_coords):
                target_lat, target_lon = gps_coords
                publish_custom_gps(target_lat, target_lon, 0.0, topic='/gps_targets')
                print("Sent message")
            else:
                print("NOTHING")

            
            #Send to RoboDog (if detected)
           
            time.sleep(3)
            print("Move to next scan area")
            print(f"Iteration: {iteration}")
            print(f"Threads: {len(detection_manager.active_threads)}")
            print(f"Results Queue {detection_manager.results_queue.qsize()}")
            print(f"Image Queue {detection_manager.image_queue.qsize()}")
            print(f"GPS Queue {detection_manager.gps_queue.qsize()}")
            iteration += 1

    except KeyboardInterrupt:
        print("KeyboardInterrupt Exit")
        print(f"Scan ended with: {numberOfScans} scans")
        print(f"GPS of scanned area Lat: {gpsStoredLat}")
        print(f"GPS of scanned area Lat: {gpsStoredLon}")
        data_manager.close()
        camera.terminate()


if __name__ == "__main__":

    main()
