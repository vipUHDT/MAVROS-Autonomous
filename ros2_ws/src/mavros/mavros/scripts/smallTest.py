from ros_utils_jetson import get_mavros_attitude, publish_custom_gps
import time
lat = lon = alt = roll = pitch = yaw = None
seconds = 5
print("Starting Search Area...")
try:
    while True:

        for i in range(seconds):
            time.sleep(1)
            print(i)
        print("Starting Scan")
        # Pull Drone GPS/Data
        attitude = get_mavros_attitude(timeout_sec=10)
        if attitude:
            lat, lon, alt, roll, pitch, yaw = attitude
        else:
            print("Error: no MAVROS messages received for attitude")
        print(lat, lon, alt, roll, pitch, yaw)

except KeyboardInterrupt:
    print("KeyboardInterrupt Exit")
