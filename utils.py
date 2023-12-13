import argparse
import time
from math import atan2, cos, radians, sin, sqrt

from pymavlink import mavutil

from dronekit import LocationGlobal, VehicleMode, connect


def get_vehicle_attrs(vehicle):
    print(f" - Autopilot Firmware version: {vehicle.version}")
    print(f" - Autopilot capabilities (supports ftp): {vehicle.capabilities.ftp}")
    print(f" - Global Location: {vehicle.location.global_frame}")
    print(
        f" - Global Location (relative altitude): {vehicle.location.global_relative_frame}"
    )
    print(f" - Local Location: {vehicle.location.local_frame}")
    print(f" - Attitude: {vehicle.attitude}")
    print(f" - Velocity: {vehicle.velocity}")
    print(f" - GPS: {vehicle.gps_0}")
    print(f" - Groundspeed: {vehicle.groundspeed}")
    print(f" - Airspeed: {vehicle.airspeed}")
    print(f" - Gimbal status: {vehicle.gimbal}")
    print(f" - Battery: {vehicle.battery}")
    print(f" - EKF OK?: {vehicle.ekf_ok}")
    print(f" - Last Heartbeat: {vehicle.last_heartbeat}")
    print(f" - Rangefinder: {vehicle.rangefinder}")
    print(f" - Rangefinder distance: {vehicle.rangefinder.distance}")
    print(f" - Rangefinder voltage: {vehicle.rangefinder.voltage}")
    print(f" - Heading: {vehicle.heading}")
    print(f" - Is Armable?: {vehicle.is_armable}")
    print(f" - System status: {vehicle.system_status.state}")
    print(f" - Mode: {vehicle.mode.name}")
    print(f" - Armed: {vehicle.armed}")


def connect_vehicle():
    parser = argparse.ArgumentParser(description="Connecting to SITL on local machine.")
    parser.add_argument("--connect", help="vehicle connection target string")
    args = parser.parse_args()

    connection_string = args.connect
    print(f"\nConnecting to vehicle on: {connection_string}")

    vehicle = connect(connection_string, wait_ready=True)

    # Get Vehicle Home location - will be `None` until first set by autopilot
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    while not vehicle.home_location:
        print(" Waiting for home location ...")

    # We have a home location.
    print(f"\n Home location: {vehicle.home_location}")
    return vehicle


def arm_and_takeoff(vehicle, alt_target):
    print("Basic pre-arm checks:")

    vehicle.mode = VehicleMode("STABILIZE")

    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)
    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != VehicleMode("GUIDED"):
        print(" Waiting for entering GUIDED mode...")
        print(f" current mode is {vehicle.mode}...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(3)
    print("Taking off!")
    vehicle.simple_takeoff(alt_target)  # Take off to target altitude

    # Altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= alt_target * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    print(f" Target altitude={alt_target} reached")
    return


def calc_distance(lat1, lon1, lat2, lon2):
    """Calculates distance between 2 coords in metres"""
    R = 6373.0

    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return R * c * 1000


def calc_estimated_time(dist, speed):
    """Calculates time needed to pass distance at speed
    :dist distance in (m)
    :speed distance in (m/s)
    """
    return int(dist // speed)


def condition_yaw(vehicle, heading, relative=False):
    """Change vehicle's yaw"""
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0,
        0,
        0,
    )  # param 5 ~ 7 not used

    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto(vehicle, lat, lon, alt, speed):
    print(f"Go to next location ({lat},{lon}), alt=%(alt)s, at speed=%(speed)s")
    if alt is None:
        alt = vehicle.location.global_relative_frame.alt

    # print(f"curr_location=({vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon})")

    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    dist = calc_distance(
        lat1=curr_lat,
        lon1=curr_lon,
        lat2=lat,
        lon2=lon,
    )
    wait_time = 6 + calc_estimated_time(dist=dist, speed=speed)

    print(f"curr_location=({curr_lat},{curr_lon})")
    print(f"Estimated distance={dist}")
    print(f"Estimated time={wait_time}")

    vehicle.simple_goto(LocationGlobal(lat=lat, lon=lon, alt=alt), airspeed=speed)

    for i in range(0, wait_time, 2):
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        alt = vehicle.location.global_relative_frame.alt
        print(f"\n - current location: ({curr_lat, curr_lon}) ")
        print(f" - current altitude: {alt} ")
        time.sleep(2)

    print(f"The location is ACHIVED ({curr_lat},{curr_lon})")
    return
