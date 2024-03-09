#!/usr/bin/env python3
import serial
import time  # Optional (required if using time.sleep() below)
from compute_angle import compute_wall_angle
import numpy as np
from simple_pid import PID
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, Attitude, AttitudeRate, PositionNedYaw)
from mavsdk.telemetry import Telemetry

ser_left = serial.Serial("/dev/ttyAMA0", 115200)
ser_right = serial.Serial("/dev/ttyAMA1", 115200)

WALL_DISTANCE = 80
HOVER_THRUST = 0.5

YAW_CTRL_P = 0.5
DST_CTRL_P = 0.1

VSPD_CTRL_P = 4
VSPD_CTRL_I = 2
VSPD_CTRL_D = 0
VSPD_MIN = -0.5
VSPD_MAX = 1

ACCEL_MIN = -2
ACCEL_MAX = 2

ref_altitude = 1.5
altitude_global = None
vertical_speed_global = None


async def get_position(drone):
    global altitude_global
    async for position in drone.telemetry.position():
        altitude_global = position.relative_altitude_m
        print("Altitude is", altitude_global)


async def get_vspeed(drone):
    global vertical_speed_global
    async for x in drone.telemetry.position_velocity_ned():
        vertical_speed_global = -x.velocity.down_m_s
        print("Vertical speed is", vertical_speed_global)


def update_data(port):
    bytes_serial = port.read(9)
    port.reset_input_buffer()
    distance = None
    if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:  # this portion is for python3
        print("Printing python3 portion")
        distance = bytes_serial[2] + bytes_serial[
            3] * 256
        strength = bytes_serial[4] + bytes_serial[5] * 256
        temperature = bytes_serial[6] + bytes_serial[7] * 256
        temperature = (temperature / 8) - 256
        # print("Distance:" + str(distance))
        # print("Strength:" + str(strength))
        port.reset_input_buffer()
    return distance


# we define a new function that will get the data from LiDAR and publish it
async def main():
    global ref_altitude
    drone = System()
    await drone.connect(system_address="serial:///dev/pixhawk")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, HOVER_THRUST))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                    with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    asyncio.ensure_future(get_position(drone))
    asyncio.ensure_future(get_vspeed(drone))

    while altitude_global is None or vertical_speed_global is None:
        print("Waiting for altitude or vspeed")
        await asyncio.sleep(1)
    ref_altitude = altitude_global

    distance_left = None
    distance_right = None
    yaw_pid_controller = PID(YAW_CTRL_P, 0, 0, output_limits=(-1, 1))
    distance_pid_controller = PID(DST_CTRL_P, 0, 0, output_limits=(-0.1, 0.1), setpoint=WALL_DISTANCE)

    altitude_controller = PID(1, 0, 0, output_limits=(VSPD_MIN, VSPD_MAX), setpoint=ref_altitude)
    vertical_speed_controller = PID(VSPD_CTRL_P, VSPD_CTRL_I, VSPD_CTRL_D, output_limits=(ACCEL_MIN, ACCEL_MAX))
    while True:
        # counter_l = ser_left.in_waiting  # count the number of bytes of the serial port
        # counter_r = ser_right.in_waiting
        # if counter_l > 8:
        #     res = update_data(ser_left)
        #     if res is not None:
        #         distance_left = res
        # if counter_r > 8:
        #     res = update_data(ser_right)
        #     if res is not None:
        #         distance_right = res
        # if distance_left is None or distance_right is None:
        #     print("Got none distance")
        #     break
        #     # continue
        # if 10 > distance_left > 200 or 10 > distance_right > 200:
        #     print("Readings out of min/max range")
        #     break
        #     # continue
        # angle_rad, wall_dist = compute_wall_angle(distance_left, distance_right, np.pi/4)
        # angle_rad -= np.pi / 2
        # angle_deg = np.rad2deg(angle_rad)
        # #print(np.rad2deg(angle_rad))
        # print("wall_dist", str(wall_dist), "angle", str(angle_deg))
        # control_effort_yaw = yaw_pid_controller(angle_deg)
        # control_effort_dist = -distance_pid_controller(wall_dist)
        # print("Yaw effort:", control_effort_yaw, "Dist effort:", control_effort_dist)

        altitude_controller.setpoint = ref_altitude
        vertical_speed_ref = altitude_controller(altitude_global)
        vertical_speed_controller.setpoint = vertical_speed_ref
        control_effort_vert_accel = vertical_speed_controller(vertical_speed_global)/10
        print("Setting attitude")
        thrust = HOVER_THRUST+control_effort_vert_accel
        print("Thrust is", thrust)
        await drone.offboard.set_attitude(Attitude(0, 2, 0, thrust))
        # await drone.offboard.set_velocity_body(
        #     VelocityBodyYawspeed(control_effort_dist, 0.0, 0.0, control_effort_yaw))
        await asyncio.sleep(0.1)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
                      {error._result.result}")


if __name__ == "__main__":
    try:
        if not ser_left.isOpen():
            ser_left.open()
        if not ser_right.isOpen():
            ser_right.open()
        asyncio.run(main())
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser_left is not None:
            ser_left.close()
        if ser_right is not None:
            ser_right.close()
        print("program interrupted by the user")
