import argparse
import time
from math import atan2, cos, radians, sin, sqrt

from dronekit import VehicleMode
from modules.drone import Drone


def connect_vehicle() -> Drone:
    parser = argparse.ArgumentParser(description="Connecting to SITL on local machine.")
    parser.add_argument("--connect", help="vehicle connection target string")
    args = parser.parse_args()

    connection_string = args.connect
    print(f"\nConnecting to vehicle on: {connection_string}")

    vehicle = Drone.connect(connection_string)

    # Get Vehicle Home location - will be `None` until first set by autopilot
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    while not vehicle.home_location:
        print(" Waiting for home location ...")

    # We have a home location.
    print(f"\n Home location: {vehicle.home_location}")
    return Drone(vehicle)


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
