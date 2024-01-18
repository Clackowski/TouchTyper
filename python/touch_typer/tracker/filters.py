import numpy as np
import math
from scipy import zeros, signal
from filterpy.kalman import KalmanFilter

from ..util.utils import *


class Filter():
    def __init__(self, lookback_count = 0) -> None:
        self.iter = 0
        self.lookback_count = lookback_count
        self.raw_data = DequeOfArrays(self.lookback_count + 1)
        self.filtered_data = DequeOfArrays(self.lookback_count + 1)

    def _need_more_samples(self):
        self.iter += 1
        return self.iter - 1 < self.lookback_count

    def reset(self) -> None:
        print("reset")
        self.iter = 0

    def filter(self, raw_data) -> np.ndarray:
        return np.array([])

class SciPyHighPassOrder(Filter):
    def __init__(self, cutoff_freq_hz, dt, order) -> None:
        super().__init__(8)
        self.cutoff_freq = cutoff_freq_hz
        self.rate = 1 / dt
        self.order = order
        nyquist_freq = 0.5 * self.rate
        # nyquist normalized cutoff for digital design
        Wn = self.cutoff_freq / nyquist_freq
        self.sos = signal.butter(order, Wn, btype='highpass', output='sos')
        self.zf = np.zeros((math.ceil(order / 2), 2, 3))

    def reset(self):
        self.zf = np.zeros((math.ceil(self.order / 2), 2, 3))

    def filter(self, raw_data):
        y, self.zf = signal.sosfilt(self.sos, np.mat(raw_data[-1]), axis=0, zi=self.zf)
        return np.array(y[-1])


class SciPyLowPassOrder(Filter):
    def __init__(self, cutoff_freq_hz, dt, order) -> None:
        super().__init__(8)
        self.cutoff_freq = cutoff_freq_hz
        self.rate = 1 / dt
        self.order = order
        nyquist_freq = 0.5 * self.rate
        # nyquist normalized cutoff for digital design
        Wn = self.cutoff_freq / nyquist_freq
        self.sos = signal.butter(order, Wn, btype='lowpass', output='sos')
        self.zf = np.zeros((math.ceil(order / 2), 2, 3))

    def reset(self):
        self.zf = np.zeros((math.ceil(self.order / 2), 2, 3))

    def filter(self, raw_data):
        y, self.zf = signal.sosfilt(self.sos, np.mat(raw_data[-1]), axis=0, zi=self.zf)
        return np.array(y[-1])


# class SciPyHighPass(Filter):
#     def __init__(self, cutoff_freq_hz, dt) -> None:
#         super().__init__(2)
#         self.cutoff_freq = cutoff_freq_hz
#         self.rate = 1 / dt
#         nyquist_freq = 0.5 * self.rate
#         # nyquist normalized cutoff for digital design
#         Wn = self.cutoff_freq / nyquist_freq
#         self.b, self.a = signal.butter(2, Wn, btype='highpass')

#     def filter(self, raw_data, final_data):
#         if self._need_more_samples():
#             raw_data = np.append([[0,0,0],[0,0,0], [0,0,0]], raw_data[-self.iter:], axis=0)
#             final_data = np.append([[0,0,0],[0,0,0],[0,0,0]], final_data[-self.iter + 1:], axis=0)
#         return -self.a[1] * final_data[-1] - self.a[2] * final_data[-2] + self.b[0] * raw_data[-1] + self.b[1] * raw_data[-2] + self.b[2] * raw_data[-3]

# class SciPyLowPass(Filter):
#     def __init__(self, cutoff_freq_hz, dt) -> None:
#         super().__init__(2)
#         self.cutoff_freq = cutoff_freq_hz
#         self.rate = 1 / dt
#         nyquist_freq = 0.5 * self.rate
#         # nyquist normalized cutoff for digital design
#         Wn = self.cutoff_freq / nyquist_freq
#         self.b, self.a = signal.butter(2, Wn, btype='lowpass')

#     def filter(self, raw_data, final_data):
#         if self._need_more_samples():
#             raw_data = np.append([[0,0,0],[0,0,0], [0,0,0]], raw_data[-self.iter:], axis=0)
#             final_data = np.append([[0,0,0],[0,0,0],[0,0,0]], final_data[-self.iter + 1:], axis=0)
#         return -self.a[1] * final_data[-1] - self.a[2] * final_data[-2] + self.b[0] * raw_data[-1] + self.b[1] * raw_data[-2] + self.b[2] * raw_data[-3]

# class ButterFilter(Filter):
#     def __init__(self) -> None:
#         super().__init__(0)

#     def filter(self, raw_data, final_data):
#         result = np.array([0,0,0])
#         for i in [0, 1, 2]:
#             if self.iter == 0:
#                 self.b = signal.firwin(40, 0.4)
#                 self.z = signal.lfilter_zi(self.b, 1) * raw_data[0][i]
#                 self.iter += 1
#             result[i], self.z = signal.lfilter(self.b, 1, [raw_data[-1][i]], zi=self.z)
#         return result

# class SimpleDCBlocker(Filter):
#     def __init__(self, omega) -> None:
#         super().__init__(1)
#         self.omega = omega
#         self.R = 1 - omega

#     def filter(self, raw_data, final_data) -> np.ndarray:
#         if self._need_more_samples():
#             return raw_data[-1]
#         return raw_data[-1] - raw_data[-2] + self.R * final_data[-1]


# class DCBlocker(Filter):
#     def __init__(self, omega) -> None:
#         super().__init__(2)
#         self.omega = omega

#     def filter(self, raw_data, final_data) -> np.ndarray:
#         if self._need_more_samples():
#             return raw_data[-1]
#         return (1 - (2**0.5) * self.omega) * (raw_data[-1] - 2*raw_data[-2] + raw_data[-3]) + (2 - (2**0.5)*self.omega - (1/2)*(self.omega**2)) * final_data[-1] - (1 - (2**0.5)*self.omega)**2 * final_data[-2]

# class HighPassFilter(Filter):
#     def __init__(self, omega) -> None:
#         super().__init__(3)
#         self.omega = omega

#     def filter(self, raw_data, final_data) -> np.ndarray:
#         if self._need_more_samples():
#             return raw_data[-1]
#         # Assuming that raw data has current tick and final / filtered has up to prev.
#         ret = np.array([0,0,0])
#         #omega = 2 * math.pi * 0.0006 / 100
#         filter = lambda ys, xs, om: (1 - om) * (xs[-1] - 3 * xs[-2] + 3 * xs[-3] - xs[-4]) + \
#                 ((6 - 7 * om) / (2 - om)) * ys[-1] - \
#                 ((6 + om) / (2 - om)) * (1 - om) * (1 - om) * ys[-2] + (1 - om) * (1 - om) * ys[-3]
#         ret = filter(final_data, raw_data, self.omega)
#         return ret


# class HighPassFilterV2(Filter):
#     def __init__(self, omega) -> None:
#         super().__init__(3)
#         self.omega = omega

#     def filter(self, raw_data, final_data) -> np.ndarray:
#         if self._need_more_samples():
#             return raw_data[-1]
#         # Assuming that raw data has current tick and final / filtered has up to prev.
#         ret = np.array([0,0,0])
#         #omega = 2 * math.pi * 0.0006 / 100
#         filter = lambda ys, xs, om: (1 - om) * (xs[-1] - 3 * xs[-2] + 3 * xs[-3] - xs[-4]) \
#                 + ((6 - 7 * om) / (2 - om)) * ys[-1] \
#                 - ((6 + om) / (2 - om)) * (1 - om) * (1 - om) * ys[-2] \
#                 + (1 - om) * (1 - om) * ys[-3]
#         ret = filter(final_data, raw_data, self.omega)
#         return ret

class KalmanFilt(Filter):
    def __init__(self) -> None:
        signal_variance = 0.001 # Guess at the variance of the signal.
        dt = 0.03

        self.kf = KalmanFilter(3, 1)

        # initial state mean
        self.kf.x = np.array([0, 0, 0])

        # transition_matrix  
        self.kf.F = np.array(
                 [[1, dt, 0.5*dt**2], 
                  [0,  1,       dt],
                  [0,  0,        1]])

        # observation_matrix   
        self.kf.H = np.array([[0., 0., 1.]])

        # initial state covariance
        self.kf.P = np.array(
                   [[0, 0, 0], 
                    [0, 0, 0],
                    [0, 0, signal_variance]])

        # observation_covariance 
        self.kf.R = signal_variance
    

        # transition_covariance 
        self.kf.Q = np.array(
                 [[0.2,    0,      0], 
                  [  0,  0.1,      0],
                  [  0,    0,  10e-4]])

    def filter(self, raw_data) -> np.ndarray:
        """yields pos, vel, acc"""
        self.kf.predict()
        self.kf.update(raw_data)

        return self.kf.x
