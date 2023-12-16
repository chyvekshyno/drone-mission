import argparse
from math import atan2, cos, radians, sin, sqrt

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
    return vehicle


def get_bearing(loc1, loc2):
    """
    Returns the bearing in radians between the two LocationGlobal objects passed as parameters.
    """
    dL = loc2.lon - loc1.lon
    X = cos(loc2.lat) * sin(dL)
    Y = cos(loc1.lat) * sin(loc2.lat) - sin(loc1.lat) * cos(loc2.lat) * cos(dL)
    bearing = atan2(X, Y)
    # if bearing < 0:
    #     bearing += 6.28318530718
    return bearing


def rad2deg(rad):
    return rad * 57.2957795


def deg2rad(deg):
    if deg > 180:
        deg = deg - 360
    return deg / 57.2957795


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
