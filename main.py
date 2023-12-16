import time

from dronekit import LocationGlobalRelative
import utils

dest_lat = 50.443326
dest_lon = 30.448078
dest_alt = 100

drone = utils.connect_vehicle()

drone.arm()
drone.take_off(dest_alt)
drone.set_mode("ALT_HOLD")
drone.goto_rc(LocationGlobalRelative(lat=dest_lat, lon=dest_lon, alt=dest_alt))
time.sleep(1)
drone.set_bearing(utils.deg2rad(350))
time.sleep(5)

drone.close()
