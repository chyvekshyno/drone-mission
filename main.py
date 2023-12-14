import time

from dronekit import LocationGlobalRelative
import utils

dest_lat = 50.443326
dest_lon = 30.448078
dest_alt = 100

drone = utils.connect_vehicle()

drone.arm_and_takeoff(dest_alt)
time.sleep(3)

drone.goto(
    location=LocationGlobalRelative(lat=dest_lat, lon=dest_lon, alt=dest_alt),
    speed=10,
)
time.sleep(3)

drone.send_movement_cmd_yaw(heading=350)
time.sleep(5)

drone.close()
