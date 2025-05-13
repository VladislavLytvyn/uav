import time
import math

import dronekit_sitl
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException


HOME_LAT, HOME_LON = 50.450739, 30.461242
TARGET_LAT, TARGET_LON = 50.443326, 30.448078
TARGET_ALT = 100


sitl = dronekit_sitl.start_default(HOME_LAT, HOME_LON)
connection_string = sitl.connection_string()

print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Takeoff to specified altitude
    """
    print("Preparing for takeoff.")

    while not vehicle.is_armable:
        print("Waiting for takeoff readiness...")
        time.sleep(1)

    print("Engines are running!")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for the engines to start...")
        time.sleep(1)

    print("Rise.")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(f"Height: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Height reached.")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """
    Calculating the distance between two points
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Calculating the azimuth between two points
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Sending speed commands in the NED (North, East, Down) system
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)


def condition_yaw(heading, relative=False):
    """
    Turn the copter to the specified azimuth (heading)
    """
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        1,
        is_relative,
        0, 0, 0
    )

    vehicle.send_mavlink(msg)

try:
    arm_and_takeoff(TARGET_ALT)
    time.sleep(5)

    vehicle.mode = VehicleMode("ALT_HOLD")
    print("Mode changed to ALT_HOLD")
    time.sleep(2)

    target_location = LocationGlobalRelative(TARGET_LAT, TARGET_LON, TARGET_ALT)

    current_location = vehicle.location.global_relative_frame

    target_bearing = get_bearing(current_location, target_location)
    print(f"Direction to point B: {target_bearing} degrees")

    target_distance = get_distance_metres(current_location, target_location)
    print(f"Distance to point B: {target_distance} meters")

    speed = 50

    flight_time = target_distance / speed
    print(f"Expected flight time: {flight_time} seconds")

    condition_yaw(target_bearing, False)
    print(f"Directional U-turn {target_bearing} degrees")
    time.sleep(3)

    print("Flight to point B...")
    send_ned_velocity(speed, 0, 0)

    start_time = time.time()

    while time.time() - start_time < flight_time:
        current_location = vehicle.location.global_relative_frame
        remaining_distance = get_distance_metres(current_location, target_location)
        print(f"It remains: {remaining_distance:.2f} meters")

        if remaining_distance < 10:
            print("Almost arrived at point B")
            break

        time.sleep(1)

    send_ned_velocity(0, 0, 0)
    print("Arrived at point B")
    time.sleep(2)

    print("Turn to azimuth 350°")
    condition_yaw(350, False)
    time.sleep(5)

    vehicle.mode = VehicleMode("LOITER")
    print("Mission accomplished, stuck in LOITER mode")
    time.sleep(5)

except APIException as e:
    print(f"APIException error: {e}")
    print("\nTips for solving the problem:")
    print("1. Make sure Mission Planner is running and the simulator is activated")
    print("2. Check if port 5760 is not occupied by another program")
    print("3. Try restarting your computer and starting over.")

except ConnectionRefusedError as e:
    print(f"Connection error: {e}")
    print("Make sure the simulator is running and listening on port 5760")

except KeyboardInterrupt:
    print("\nOperation aborted by user")

except Exception as e:
    print(f"Unexpected error: {e}")
    print("Try restarting Mission Planner and running the script again.")

finally:
    print("Script termination")
    vehicle.close()
    print("Connection completed")
