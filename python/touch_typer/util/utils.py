from collections import deque
from dataclasses import dataclass
import numpy as np
from typing import List

from .quats import *

DATA_BUFFER_SIZE = 100000


class DequeOfArrays():
    def __init__(self, size = None) -> None:
        self.size = size or DATA_BUFFER_SIZE
        self.data = deque(maxlen=self.size)
    
    def __getitem__(self, index):
        return self.data[index]
    
    def __setitem__(self, index, value):
        self.data[index] = value
    
    def push(self, array):
        self.data.append(array)


@dataclass
class Data:
    time: List[float]
    raw_accel: np.ndarray
    linear_accel: np.ndarray
    final_accel: np.ndarray
    unfiltered_velocity: np.ndarray
    final_velocity: np.ndarray
    unfiltered_position: np.ndarray
    final_position: np.ndarray
    finger_position: np.ndarray
    raw_gyro: np.ndarray
    raw_magnetometer: np.ndarray
    orientation: List[Quaternion]

    def printable(self):
        return f"--------------------\ntime: {self.time[-1]}\n" \
                f"raw_acc:    {self.raw_accel[-1]}\n" \
                f"lin_acc:    {self.linear_accel[-1]}\n" \
                f"final_acc:  {self.final_accel[-1]}\n" \
                f"vel:        {self.unfiltered_velocity[-1]}\n" \
                f"final_vel:  {self.final_velocity[-1]}\n" \
                f"pos:        {self.unfiltered_position[-1]}\n" \
                f"final_pos:  {self.final_position[-1]}\n" \
                f"finger_pos: {self.final_position[-1]}\n" \
                f"gyro:       {self.raw_gyro[-1]}\n" \
                f"mag:        {self.raw_magnetometer[-1]}\n" \
                f"or:         {self.orientation[-1]}\n--------------------------\n"

    def push_time(self, time: float):
        if len(self.time) == 0:
            self.time = [time]
        else:
            self.time.append(time)

    def push_raw_accel(self, data: np.ndarray):
        if self.raw_accel.size == 0:
            self.raw_accel = np.array([data])
        else:
            self.raw_accel = np.append(self.raw_accel, [data], axis=0)

    def push_linear_accel(self, data: np.ndarray):
        if self.linear_accel.size == 0:
            self.linear_accel = np.array([data])
        else:
            self.linear_accel = np.append(self.linear_accel, [data], axis=0)

    def push_final_accel(self, data: np.ndarray):
        if self.final_accel.size == 0:
            self.final_accel = np.array([data])
        else:
            self.final_accel = np.append(self.final_accel, [data], axis=0)

    def push_unfiltered_velocity(self, data: np.ndarray):
        if self.unfiltered_velocity.size == 0:
            self.unfiltered_velocity = np.array([data])
        else:
            self.unfiltered_velocity = np.append(self.unfiltered_velocity, [data], axis=0)

    def push_final_velocity(self, data: np.ndarray):
        if self.final_velocity.size == 0:
            self.final_velocity = np.array([data])
        else:
            self.final_velocity = np.append(self.final_velocity, [data], axis=0)

    def push_unfiltered_position(self, data: np.ndarray):
        if self.unfiltered_position.size == 0:
            self.unfiltered_position = np.array([data])
        else:
            self.unfiltered_position = np.append(self.unfiltered_position, [data], axis=0)

    def push_final_position(self, data: np.ndarray):
        if self.final_position.size == 0:
            self.final_position = np.array([data])
        else:
            self.final_position = np.append(self.final_position, [data], axis=0)

    def push_raw_gyro(self, data: np.ndarray):
        if self.raw_gyro.size == 0:
            self.raw_gyro = np.array([data])
        else:
            self.raw_gyro = np.append(self.raw_gyro, [data], axis=0)

    def push_raw_magnetometer(self, data: np.ndarray):
        if self.raw_magnetometer.size == 0:
            self.raw_magnetometer = np.array([data])
        else:
            self.raw_magnetometer = np.append(self.raw_magnetometer, [data], axis=0)

    def push_finger_position(self, data: np.ndarray):
        if self.finger_position.size == 0:
            self.finger_position = np.array([data])
        else:
            self.finger_position = np.append(self.finger_position, [data], axis=0)

    def drop_oldest_data(self):
        self.time = self.time[-DATA_BUFFER_SIZE:]
        self.raw_accel = np.array(self.raw_accel)[-DATA_BUFFER_SIZE:]
        self.linear_accel = np.array(self.linear_accel)[-DATA_BUFFER_SIZE:]
        self.final_accel = np.array(self.final_accel)[-DATA_BUFFER_SIZE:]
        self.unfiltered_velocity = np.array(self.unfiltered_velocity)[-DATA_BUFFER_SIZE:]
        self.final_velocity = np.array(self.final_velocity)[-DATA_BUFFER_SIZE:]
        self.unfiltered_position = np.array(self.unfiltered_position)[-DATA_BUFFER_SIZE:]
        self.final_position = np.array(self.final_position)[-DATA_BUFFER_SIZE:]
        self.raw_gyro = np.array(self.raw_gyro)[-DATA_BUFFER_SIZE:]
        self.raw_magnetometer = np.array(self.raw_magnetometer)[-DATA_BUFFER_SIZE:]
        self.orientation = self.orientation[-DATA_BUFFER_SIZE:]
        self.finger_position = self.finger_position[-DATA_BUFFER_SIZE:]


    def to_dict_with_most_recent_data(self, reverse_ys=False) -> dict:
        
        return {'x_pos':-self.finger_position[-1][0], 
                'y_pos': self.finger_position[-1][1] if reverse_ys else -self.finger_position[-1][1], 
                'z_pos':self.finger_position[-1][2], 
                'x_vel':self.final_velocity[-1][0], 
                'y_vel':-self.final_velocity[-1][1] if reverse_ys else self.final_velocity[-1][1], 
                'z_vel':self.final_velocity[-1][2], 
                'x_acc':   self.final_accel[-1][0], 
                'y_acc':   -self.final_accel[-1][1] if reverse_ys else self.final_accel[-1][1], 
                'z_acc':   self.final_accel[-1][2], }


def create_calibration(data: Data, num_samples) -> np.ndarray:
    sampled_accel = data.raw_accel[-num_samples:]
    sampled_gyro = data.raw_gyro[-num_samples:]
    sampled_mag = data.raw_magnetometer[-num_samples:]

    accel_means = np.mean(sampled_accel, axis=0)
    gyro_means = np.mean(sampled_gyro, axis=0)
    mag_means = np.mean(sampled_mag, axis=0)
    
    return np.append(np.append(accel_means, gyro_means), mag_means)