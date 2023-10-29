import serial
import time  # Optional (required if using time.sleep() below)
from compute_angle import compute_wall_angle
import numpy as np
from simple_pid import PID

# Checked with TFmini plus

# ser = serial.Serial("/dev/ttyUSB1", 115200)

ser_left = serial.Serial("/dev/ttyAMA0", 115200)
ser_right = serial.Serial("/dev/ttyAMA0", 115200)
# ser = serial.Serial("COM12", 115200)

YAW_CTRL_P = 1
DST_CTRL_P = 0.01


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
        print("Distance:" + str(distance))
        print("Strength:" + str(strength))
        if temperature != 0:
            print("Temperature:" + str(temperature))
        port.reset_input_buffer()
    return distance


# we define a new function that will get the data from LiDAR and publish it
def main():
    distance_left = None
    distance_right = None
    yaw_pid_controller = PID(YAW_CTRL_P, 0, 0, output_limits=(-1, 1))
    distance_pid_controller = PID(DST_CTRL_P, 0, 0, output_limits=(-0.1, 0.1))
    while True:
        counter_l = ser_left.in_waiting  # count the number of bytes of the serial port
        counter_r = ser_right.in_waiting
        if counter_l > 8:
            res = update_data(ser_left)
            if res is not None:
                distance_left = res
        if counter_r > 8:
            res = update_data(ser_right)
            if res is not None:
                distance_right = res
        if distance_left is None or distance_right is None:
            continue
        angle_rad, wall_dist = compute_wall_angle(distance_left, distance_right, np.pi/2)
        angle_rad -= np.pi / 2
        print(np.rad2deg(angle_rad))
        control_effort_yaw = yaw_pid_controller(angle_rad)
        control_effort_dist = distance_pid_controller(wall_dist)
        print("Yaw effort:", control_effort_yaw, "Dist effort:", control_effort_dist)


if __name__ == "__main__":
    try:
        if not ser_left.isOpen():
            ser_left.open()
        if not ser_right.isOpen():
            ser_right.open()
        main()
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser_left is not None:
            ser_left.close()
        if ser_right is not None:
            ser_right.close()
        print("program interrupted by the user")