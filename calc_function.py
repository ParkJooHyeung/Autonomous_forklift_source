import numpy as np
import math


def remove_outliers(arr, threshold=1.5):
    q1 = np.percentile(arr, 25)
    q3 = np.percentile(arr, 75)
    iqr = q3 - q1
    lower_bound = q1 - threshold * iqr
    upper_bound = q3 + threshold * iqr

    filtered_arr = arr[(arr >= lower_bound) & (arr <= upper_bound)]
    return filtered_arr


def calc_distance2time(distance):
    d = distance
    time = (-1.1*10**(-8)*d**4 + 5.22*10**(-6)*d**3
            - 8.797*10**(-4)*d**2 + 1.079*10**(-1)*d + 1.9)
    return time


def calc_angle2time(angle):
    a = angle
    time = -38.68+9.08*math.log(a+94.12) # 90 -> 20.56 45-> -19.21
    return time


def calc_p_distance(center_x, depth):
    p_distance = 1.792/1280 * abs(640-center_x) * depth / 1.88
    return p_distance
