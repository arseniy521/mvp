#!/usr/bin/env python3

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, Attitude, AttitudeRate, PositionNedYaw)
import asyncio
from simple_pid import PID


async def landing():
    drone = System()
    attitude_pid = PID(1, 0, 0, output_limits=(-1, 1))
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
                  {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- STARTING CONTROL")
    print("-- Turning")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 90.0))
    await asyncio.sleep(1)
    distance = 3
    counter = 0
    print("-- Starting control loop")
    while counter < 1000:
        error_m = distance
        effort_m_s = attitude_pid(error_m)
        print(effort_m_s)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(effort_m_s, 0.0, 0.0, 0.0))
        distance -= 0.1
        distance = max(distance, 0)
        counter += 1
        await asyncio.sleep(0.1)
    # await asyncio.sleep(1)

    print("-- Finished control, setting zero speeds")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
                  {error._result.result}")

if __name__ == '__main__':
    asyncio.run(landing())

