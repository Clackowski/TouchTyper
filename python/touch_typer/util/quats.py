import math
import numpy as np


class Quaternion():
    def __init__(self, w: float = 0, x: float = 0, y: float = 0, z: float = 0) -> None:
        self.q = np.array([w, x, y, z])

    def __str__(self) -> str:
        return f"{self[0]:.6f} {self[1]:.6f} {self[2]:.6f} {self[3]:.6f}"

    def __getitem__(self, index):
        return self.q[index]

    def __mul__(self, other):
        scalar = self[0] * other[0] - np.dot(self[1:4], other[1:4])
        vector = self[0] * other[1:4] + \
                other[0] * self[1:4] + \
                np.cross(self[1:4], other[1:4])
        return Quaternion(scalar, vector[0], vector[1], vector[2])

    def normalized(self):
        norm = np.linalg.norm(self.q)
        if norm == 0:
            print("Norm in quaternion is zero, bad news.")
            return self
        return Quaternion(*(self.q / norm))

    def conj(self):
        return Quaternion(self[0], -self[1], -self[2], -self[3])

    def rotate(self, vector):
        return np.array((self * np.array([0, *vector]) * self.conj())[1:4])

    def as_numpy_array(self):
        return np.array([self[0], self[1], self[2], self[3]])

def angle_between_vectors(v1, v2) -> Quaternion:
    w = math.sqrt(sum([elt**2 for elt in v1]) * sum([elt**2 for elt in v2])) + np.dot(v1, v2)
    cross = np.cross(v2, v1)
    rotation_quat = Quaternion(w, *cross)
    if np.linalg.norm(rotation_quat.q) == 0:
        print("Norm in quat for angle_between_vectors is zero.")
    rotation_quat_normalized = rotation_quat.normalized()
    return rotation_quat_normalized

def from_roll_pitch_yaw(roll, pitch, yaw) -> Quaternion:
    # Roll about the x axis
    # Pitch about the y axis
    # Yaw about the z axis
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Quaternion(w, x, y, z)

def to_roll_pitch_yaw(q: Quaternion):
    roll = -math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
    pitch = math.asin(2 * (q[0] * q[2] - q[3] * q[1]))
    yaw = -math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) - np.pi/2
    return (roll, pitch, yaw)
