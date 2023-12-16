from typing import Any, Union, List
import time

from dronekit import (
    Vehicle,
    VehicleMode,
    LocationGlobalRelative,
    connect,
)
from pymavlink import mavutil
import utils


class Drone(Vehicle):
    @classmethod
    def connect(cls, connection_string: str = "127.0.0.1:14551"):
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.wait_for_alt
        return cls(vehicle._handler)

    #
    def disconnect(self):
        self.close()

    #
    def get_location(self):
        return self.location.global_frame

    #
    def get_altitude(self):
        return self.location.global_relative_frame.alt

    #
    def set_param(self, param: str, value: Any):
        self.parameters[param] = value

    #
    def set_mode(self, mode: Union[str, VehicleMode]):
        if isinstance(mode, str):
            mode = VehicleMode(mode)
        self.mode = mode
        while self.mode != mode:
            print(f" > Entering {mode.name} mode...")
            time.sleep(1)
        print(f"Current MODE: {self.mode}")

    def channel_ovveride(self, channel, value):
        self.channels.overrides[channel] = value

    #
    def set_roll(self, throttle: int):
        self.channel_ovveride(channel="1", value=throttle)

    #
    def set_pitch(self, throttle: int):
        self.channel_ovveride(channel="2", value=throttle)

    #
    def set_throttle(self, throttle: int):
        self.channel_ovveride(channel="3", value=throttle)

    #
    def set_yaw(self, throttle: int):
        self.channel_ovveride(channel="4", value=throttle)

    #
    def arm(self, mode: Union[str, VehicleMode] = VehicleMode("STABILIZE")):
        print("Basic pre-arm checks")
        while not self.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        print("Arming motors")

        self.set_mode(mode)

        self.armed = True
        while not self.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print("ARMED! Ready to take OFF")
        return

    #
    def take_off(self, aTargetAltitude: float):
        print(f"---\nTAKING OFF! Target altitude: {aTargetAltitude}")
        self.change_alt(aTargetAltitude)
        return

    #
    def change_alt(self, altitude: float):
        throttle = 1550
        self.set_throttle(throttle)

        while self.get_altitude() < altitude * 0.95:  # pyright: ignore
            self.set_throttle(throttle)
            time.sleep(1)
            print(f"   Altitude: {self.get_altitude()}")
        print("REACHED target altitude")

        self.set_throttle(1500)
        return

    #
    def arm_and_takeoff(
        self,
        aTargetAltitude: float,
        mode: Union[str, VehicleMode] = VehicleMode("GUIDED"),
    ):
        self.arm(mode)

        print(f"---\nTAKING OFF! Target altitude: {aTargetAltitude}")
        self.simple_takeoff(aTargetAltitude)

        # Wait until the vehicle reaches a required height
        req_alt = aTargetAltitude * 0.95
        while self.get_altitude() <= req_alt:  # pyright: ignore
            time.sleep(1)
            print(f"  > Altitude: {self.get_altitude()}")
        print("REACHED target altitude")
        return

    def goto_rc(self, location: LocationGlobalRelative):
        """
        Go to next location using RC overrides.
        Works for ALT_HOLD mode.
        """
        print(f"---\nGOTO next location ({location.lat},{location.lon})")
        print(f"Drone location: ({self.get_location().lat},{self.get_location().lon})")

        # set yaw to target
        bearing = utils.get_bearing(self.get_location(), location)
        self.set_bearing(bearing)
        print(f"  > BEARING: {bearing}")

        # calculates distance and time
        dist = utils.calc_distance(
            lat1=self.get_location().lat,
            lon1=self.get_location().lon,
            lat2=location.lat,
            lon2=location.lon,
        )
        wait_time = 2 + utils.calc_estimated_time(dist=dist, speed=10)
        print(f"  > Estimated distance: {dist}")
        print(f"  > Estimated time: {wait_time}")
        print("START MOVING")
        # move forvard
        for i in range(0, wait_time, 2):
            self.set_pitch(500)
            print(f"  > Location: ({self.get_location().lat, self.get_location().lon})")
            print(f"  > Altitude: {self.get_altitude()}\n")
            time.sleep(2)
        print(f"Location ACHIVED ({self.get_location().lat},{self.get_location().lon})")

        # stop moving forward
        self.set_pitch(1500)
        return

    #
    def goto(self, location: LocationGlobalRelative, speed):
        """
        Go to location (lat, lon) using Vehicle.simple_goto method.
        Works for GUIDED mode
        """
        print(f"---\nGOTO next location ({location.lat},{location.lon})")
        if location.alt is None:
            location.alt = self.get_altitude()

        # print(f"curr_location=({vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon})")

        dist = utils.calc_distance(
            lat1=self.get_location().lat,
            lon1=self.get_location().lon,
            lat2=location.lat,
            lon2=location.lon,
        )
        wait_time = 6 + utils.calc_estimated_time(dist=dist, speed=speed)

        print(f" - Location: ({self.get_location().lat},{self.get_location().lon})")
        print(f" - Estimated distance: {dist}")
        print(f" - Estimated time: {wait_time}")

        self.simple_goto(location=location, airspeed=speed)

        for i in range(0, wait_time, 2):
            print(
                f"\n - location: ({self.get_location().lat, self.get_location().lon})"
            )
            print(f" - altitude: {self.get_altitude()} ")
            time.sleep(2)

        print(f"Location ACHIVED ({self.get_location().lat},{self.get_location().lon})")

        return

    #
    def land(self):
        self.set_mode("LAND")

    #
    def set_bearing(self, bearing):
        # 1 = cw, -1 = ccw
        PI = 3.14159265359
        bearing2 = bearing > 0 and bearing or bearing + PI
        curr_yaw = self.attitude.yaw + PI  # pyright: ignore
        diff = curr_yaw - bearing2

        if 0 < diff and diff < PI:
            vec = -1
        else:
            vec = 1

        print(f" - set YAW: {bearing}")

        # rotate yaw
        if vec > 0:
            while curr_yaw < bearing2:  # pyright: ignore
                self.set_yaw(1530)
                print(f"\n   -> current YAW: {self.attitude.yaw}")
                print(f"   -> settet YAW: {bearing2}")
                time.sleep(1)
                curr_yaw = self.attitude.yaw + PI  # pyright: ignore
        else:
            while curr_yaw > bearing2:  # pyright: ignore
                self.set_yaw(1470)
                print(f"\n   -> current YAW: {self.attitude.yaw}")
                print(f"   -> settet YAW: {bearing2}")
                curr_yaw = self.attitude.yaw + PI  # pyright: ignore
                time.sleep(1)

        # stop rotation yaw
        self.set_yaw(1500)
        return

    #
    def send_movement_cmd_yaw(self, heading, relative=False):
        """Send cmd via mavlink to yaw drone
        :heading degrees to yaw the drone
        :relative
        """
        print(" >> Sending YAW movement command with heading: %f" % heading)

        speed = 0
        direction = 1  #  -1 ccw, 1 cw
        is_relative = relative and 1 or 0
        if heading < 0:
            direction = -1
            heading = relative and heading * -1 or 360 + heading

        # point drone into correct heading
        msg = self.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            speed,  # speed deg/s
            direction,
            is_relative,  # relative offset 1
            0,
            0,
            0,
        )

        self.send_mavlink(msg)
        return

    #
    def send_movement_cmd_xya(self, velocity_x, velocity_y, altitude):
        """Send cmd via mavlink for moving in 3D axis
        :velocity_x positive = forward, negative = backwards
        :velocity_y positive = right, negative = left
        :velocity_z positive = down, negative = up
        """

        print(
            f" >> Sending XYA movement command with x-velocity :{velocity_x}, y-velocity: {velocity_y}"
        )

        msg = self.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # relative to drone heading pos relative to EKF origin
            0b0000111111100011,  # ignore velocity z and other pos arguments
            0,
            0,
            altitude,
            velocity_x,
            velocity_y,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self.send_mavlink(msg)
        return

    #
    def get_channels_state(self, channels: Union[List[str], None] = None) -> str:
        if channels is None:
            channels = [ch for _, ch in enumerate(self.channels)]
        channels_state = "Drone's channels:"
        for ch in channels:
            channels_state += f"\n CH_{ch}: {self.channels[ch]}"
        return channels_state

    #
    def get_state(self) -> str:
        return (
            "Drone's state:"
            f"\n - Autopilot Firmware version: {self.version}"
            f"\n - Autopilot capabilities (supports ftp): {self.capabilities.ftp}"
            f"\n - Global Location: {self.location.global_frame}"
            f"\n - Global Location (relative altitude): {self.location.global_relative_frame}"
            f"\n - Local Location: {self.location.local_frame}"
            f"\n - Attitude: {self.attitude}"
            f"\n - Velocity: {self.velocity}"
            f"\n - GPS: {self.gps_0}"
            f"\n - Groundspeed: {self.groundspeed}"
            f"\n - Airspeed: {self.airspeed}"
            f"\n - Gimbal status: {self.gimbal}"
            f"\n - Battery: {self.battery}"
            f"\n - EKF OK?: {self.ekf_ok}"
            f"\n - Last Heartbeat: {self.last_heartbeat}"
            f"\n - Rangefinder: {self.rangefinder}"
            f"\n - Rangefinder distance: {self.rangefinder.distance}"
            f"\n - Rangefinder voltage: {self.rangefinder.voltage}"
            f"\n - Heading: {self.heading}"
            f"\n - Is Armable?: {self.is_armable}"
            f"\n - System status: {self.system_status.state}"
            f"\n - Mode: {self.mode.name}"  # pyright: ignore
            f"\n - Armed: {self.armed}"
        )

    #
    def __repr__(self) -> str:
        return self.get_state()
