import time
import utils

dest_lat = 50.443326
dest_lon = 30.448078
dest_alt = 100

drone = utils.connect_vehicle()

utils.arm_and_takeoff(drone, alt_target=dest_alt)
time.sleep(3)

utils.goto(drone, lat=dest_lat, lon=dest_lon, speed=10, alt=dest_alt)
time.sleep(3)

utils.condition_yaw(drone, 350)
time.sleep(5)

drone.close()
