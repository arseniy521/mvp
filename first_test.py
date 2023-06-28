#!/usr/bin/env python3

import sys
import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect()

    await drone.action.arm()

    option = sys.argv[1]

    if option == "takeoff":
        print("takeoff initiated")
        await drone.action.takeoff()
    elif option == "land":
        print("landing initiated")
        await drone.action.land()
    else:
        print("wtf")


if __name__ == "__main__":
    asyncio.run(run())
