import numpy as np
#from simple_pid import PID


def compute_third_side(angle, left_side, right_side):
    return np.sqrt(left_side**2 + right_side**2 - 2 * left_side * right_side * np.cos(angle))


def compute_dist_to_wall(a, b, c):
    wall_dist = np.sqrt(a * b * (a + b + c) * (a + b - c)/(a + b)**2)
    return wall_dist


def compute_a1(a, b, c):
    a1 = a * c / (a + b)
    return a1


def compute_alpha(a, b, c):
    cos_alpha = (b**2 + c**2 - a**2) / (2 * b * c)
    alpha = np.arccos(cos_alpha)
    return alpha


def compute_wall_angle(left_side, right_side, sensors_angle):
    c = compute_third_side(sensors_angle, left_side, right_side)
#    print("Third side is", c)
    a = left_side
    b = right_side
    wall_dist = compute_dist_to_wall(a, b, c)
#    print("Wall dist is", wall_dist)
    a1 = compute_a1(a, b, c)
    angle = compute_alpha(a, a1, wall_dist)
    return angle, wall_dist


if __name__ == '__main__':
    angle_rad, _ = compute_wall_angle(3, 3, np.pi/2)
    angle_rad -= np.pi/2
 #   yaw_pid_controller = PID(1, 0, 0, output_limits=(-1, 1))
  #  control_effort = yaw_pid_controller(angle_rad)

   # print(np.rad2deg(angle_rad))



