import ahrs
from typing import List, Optional
import numpy as np

from .filters import *
from .input import *
from .output import *
from .waveforms import *
from ..util.quats import *
from ..util.utils import *


cumulative_roll = 0

class Tracker():
    def __init__(
            self,
            acceleration_filter: Optional[Filter] = None,
            velocity_filter: Optional[Filter] = None,
            position_filter: Optional[Filter] = None,
            debug: bool = False,
            name: str = "",
            accel_bias_vector: List[float] = [1, 1, 1, 0, 0, 0],
            gyro_bias_vector: List[float] = [0, 0, 0],
            keysize: float = 0.018
        ):
        self.iter = 0
        self.debug = debug
        self.timestamp = 0
        self.name = name

        self.orientation_quat = Quaternion(1, 0, 0, 0)
        self.accel_cal_vector = np.array([0, 0, 0])
        self.gyro_cal_vector = np.array([0, 0, 0])

        self.accel_bias_vector = np.array(accel_bias_vector).reshape((2, 3))
        self.gyro_bias_vector = np.array(gyro_bias_vector)
        self.data = Data([], np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), [])

        self.acceleration_filter = acceleration_filter
        self.velocity_filter = velocity_filter
        self.position_filter = position_filter

        # TODO ideally we will not rely on thresholding
        self.accel_thresh = 0.2
        self.vel_thresh = 0.05
        self.gyro_thresh = 0.1

        self.fingertip_dist = - (keysize / 2)


    def toggle_debug(self):
        self.debug = not self.debug
        return self.debug


    def start(self, calibration) -> None:
        self.calibrate(calibration)
        self.data.push_time(self.timestamp)
        self.data.push_raw_accel(calibration[:3])
        self.data.push_linear_accel(np.array([0,0,0]))
        self.data.push_final_accel(np.array([0,0,0]))
        self.data.push_unfiltered_velocity(np.array([0,0,0]))
        self.data.push_final_velocity(np.array([0,0,0]))
        self.data.push_unfiltered_position(np.array([0,0,0]))
        self.data.push_final_position(np.array([0,0,0]))
        self.data.push_raw_gyro(np.array([0,0,0]))
        self.data.push_raw_magnetometer(np.array([0,0,0]))
        self.data.orientation.append(self.orientation_quat)
        self.data.push_finger_position(np.array([0,0,0]))


    def calibrate(self, calibration) -> None:
        # Figure out the magnitude of the gravity vector to cancel in every loop
        # as well as the initial angle of the device.
        accel_cal_vector = calibration[:3]
        expected_accel_vector = np.array([0, 0, 9.8])
        self.calibrated_orientation_quat = angle_between_vectors(expected_accel_vector, accel_cal_vector) * Quaternion(1, 0, 0, 0)
        self.orientation_quat = self.calibrated_orientation_quat
        self.accel_cal_vector = accel_cal_vector
        self.gravity_cal_vector = np.array([0, 0, np.linalg.norm(accel_cal_vector)])

        self.madgwickIMU = ahrs.filters.madgwick.Madgwick()
        self.madgwickMARG = ahrs.filters.madgwick.Madgwick()
        self.madgwickMARG.q0 = self.orientation_quat.as_numpy_array()
        self.orientation_MARG = self.orientation_quat.as_numpy_array()
        self.orientation_IMU = self.orientation_quat.as_numpy_array()

        self.gyro_cal_vector = calibration[3:]


    def reset(self, calibration = None, finger_position = None) -> None:
        for filter in [self.acceleration_filter, self.velocity_filter, self.position_filter]:
            if filter:
                filter.reset()

        if calibration is not None:
            self.start(calibration)

        self.data.unfiltered_velocity[-1] = np.array([0,0,0])
        self.data.final_velocity[-1] = (np.array([0,0,0]))
        if finger_position is not None:
            fingertip_offset = self.orientation_quat.rotate([0,0,self.fingertip_dist])
            self.data.unfiltered_position[-1] = finger_position - fingertip_offset
            self.data.final_position[-1] = finger_position - fingertip_offset
            self.data.finger_position[-1] = finger_position


    def run_iteration(self, new_data, override_dt: Optional[int] = None) -> Data:
        new_timestamp = new_data[0]
        dt = override_dt or (new_timestamp - self.timestamp)
        if dt <= 0 or dt > 1: # Control against very abnormal time differences.
            dt = 0.03
        self.timestamp = new_timestamp
        self.data.push_time(new_timestamp)
        accel = self.do_acc_bias_update(new_data[1:4], self.accel_bias_vector)
        gyro = self.do_gyro_bias_update(new_data[4:7], self.gyro_bias_vector)
        mag = new_data[7:]
        self.do_rotation_update(dt, gyro, accel, mag)
        self.do_position_update(dt, *accel)
        self.data.drop_oldest_data()

        if self.debug:
            print(self.name)
            print("dt:", dt)
            print(self.data.printable())

        self.iter += 1

        return self.data
    
    def do_acc_bias_update(self, acc, bias):
        return np.add(np.multiply(bias[0], acc), bias[1])
    
    def do_gyro_bias_update(self, gyro, bias):
        return np.subtract(gyro, bias)

    def do_rotation_update(self, dt, gyro, acc, mag) -> None:
        global cumulative_roll
        roll, pitch, yaw = gyro[0], gyro[1], gyro[2]
        self.data.push_raw_gyro(np.array([roll, pitch, yaw]))
        self.data.push_raw_magnetometer(np.array(mag))
        roll -= self.gyro_cal_vector[0]
        pitch -= self.gyro_cal_vector[1]
        yaw -= self.gyro_cal_vector[2]
        if abs(roll) < self.gyro_thresh:
            roll = 0
        if abs(pitch) < self.gyro_thresh:
            pitch = 0
        if abs(yaw) < self.gyro_thresh:
            yaw = 0
        roll *= dt
        pitch *= dt
        yaw *= dt
        rotation_quat = from_roll_pitch_yaw(roll, pitch, yaw)
        if self.debug:
            cumulative_roll += roll
            print("debug rotation---")
            print("cumulative roll:", cumulative_roll)
            print("rpy in rad", roll, pitch, yaw)
            print(rotation_quat)
            print("prev orientation:", self.orientation_quat)
        self.orientation_quat = self.orientation_quat * rotation_quat

        if self.debug:
            print("new orientation:", self.orientation_quat)

        self.data.orientation.append(self.orientation_quat)

        self.madgwickIMU.Dt = dt
        self.madgwickMARG.Dt = dt
        self.orientation_MARG = self.madgwickMARG.updateMARG(self.orientation_MARG, gyro, acc, 1000 * mag)
        self.orientation_IMU = self.madgwickIMU.updateIMU(self.orientation_IMU, gyro, acc)
        if self.debug:
            print("MARG:", self.orientation_MARG)
            print("IMU:", self.orientation_IMU)
            print()

        if self.iter > 10:
            self.orientation_quat = Quaternion(*self.orientation_IMU)
            self.data.orientation[-1] = self.orientation_quat


    def do_position_update(self, dt, accel_x, accel_y, accel_z) -> None:
        acceleration = np.array([accel_x, accel_y, accel_z])
        self.data.push_raw_accel(acceleration)

        rotation_quat = self.orientation_quat.normalized()
        rotated_acceleration = rotation_quat.rotate(acceleration)
        linear_acceleration = rotated_acceleration - self.gravity_cal_vector
    
        # Account for centripetal acceleration that shows up in the accelerometers
        gyro = self.data.raw_gyro[-1]
        vel = self.data.unfiltered_velocity[-1]
        centripetal_acceleration = np.matmul([[0, -gyro[2], gyro[1]],
                                              [gyro[2], 0, -gyro[0]],
                                              [-gyro[1], gyro[0], 0]],
                                              vel)
        linear_acceleration -= centripetal_acceleration
        

        if self.debug:
            print("debug accelerations---")
            print(acceleration)
            print(rotated_acceleration)
            print(linear_acceleration)
            print("ca", centripetal_acceleration)

        # RULE: if accel under a threshold, then act as if no acceleration.
        for n in range(3):
            if (abs(linear_acceleration[n]) < self.accel_thresh):
                linear_acceleration[n] = 0

        self.data.push_linear_accel(linear_acceleration)

        final_accel = linear_acceleration.copy()
        if (filter := self.acceleration_filter):
            final_accel = filter.filter(self.data.linear_accel)
        self.data.push_final_accel(final_accel)





        # Update velocity using rotated acceleration and delta time
        delta_vel = final_accel * dt

        current_vel = self.data.unfiltered_velocity[-1] if self.iter > 0 else np.array([0,0,0])
        new_current_vel = current_vel + delta_vel

        self.data.push_unfiltered_velocity(new_current_vel)

        final_velocity = self.data.unfiltered_velocity[-1].copy()
        if (filter := self.velocity_filter):
            final_velocity = filter.filter(self.data.unfiltered_velocity)

        # RULE: if velocity under a threshold, or no acceleration at current moment, then act as if no velocity.
        for n in range(3):
            if (abs(final_velocity[n]) < self.vel_thresh) or (linear_acceleration[n] == 0):
                final_velocity[n] = 0

        self.data.push_final_velocity(final_velocity)

        # Update position using velocity and delta time.
        delta_pos = final_velocity * dt
        current_pos = self.data.unfiltered_position[-1]
        self.data.push_unfiltered_position(current_pos + delta_pos)

        final_position = self.data.unfiltered_position[-1].copy()
        if (filter := self.position_filter):
            final_position = filter.filter(self.data.unfiltered_position)

        self.data.push_final_position(final_position)

        fingertip_offset = self.orientation_quat.rotate([0,0,self.fingertip_dist]) # The fingertip is directly below the IMU (in the frame of the IMU).
        finger_position = final_position + fingertip_offset
        self.data.push_finger_position(finger_position)
