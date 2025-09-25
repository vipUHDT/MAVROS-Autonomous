from ros_utils_jetson import get_mavros_gps, publish_custom_gps
coords = get_mavros_gps(timeout_sec=10)
# Returns tuple (lat, lon, alt)
if coords:
    lat, lon, alt = coords
    print(lat, lon)

    publish_custom_gps(lat, lon, topic='/gps_targets')
else:
    print("No GPS fix in time")

