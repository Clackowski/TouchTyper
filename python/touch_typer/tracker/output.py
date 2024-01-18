import matplotlib.pyplot as plt
from typing import List, Optional, Callable
from vpython import *
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from ..util.utils import *
from ..keypress.keymap import *


class OutputStream():
    def __init__(self, batchsize: int = 1) -> None:
        self._batchsize = batchsize
        self._iter = 0

    def put_metadata(self, tracker_indices, calibrations) -> None:
        pass

    def put_new_data(self, data: List[Data]) -> None:
        self._iter = (self._iter + 1) % self._batchsize
        if self._iter == 0:
            self._process_data(data)

    # Actually put data in a file or graph or send to vpython or whatever.
    def _process_data(self, data: List[Data]) -> None:
        pass


class VisualOutputStream(OutputStream):
    def __init__(self, num, keymap: KeyMap, key_offsets, tracker_indices) -> None:
        super().__init__()

        self.debugging = False
        self.num = num
        self.scale = keymap.key_size

        self.key_offsets = key_offsets

        self.tracker_indices = tracker_indices

        scene.range = self.scale * 8
        scene.center = vector(self.scale * 8, -self.scale * 4, 0)
        scene.background = color.white
        scene.width = 1200
        scene.height = 1080

        self.display_keyboard(keymap)

        self.xarrow = arrow(length=self.scale, shaftwidth=0.1 * self.scale, color=color.red, axis=vector(1,0,0), pos=vector(0,0,0)  * self.scale)
        self.yarrow = arrow(length=self.scale, shaftwidth=0.1 * self.scale, color=color.green, axis=vector(0,1,0), pos=vector(0,0,0)  * self.scale)
        self.zarrow = arrow(length=self.scale, shaftwidth=0.1 * self.scale, color=color.blue, axis=vector(0,0,1), pos=vector(0,0,0)  * self.scale)
        self.frontArrows = []
        self.upArrows = []
        self.sideArrows = []
        self.imuObjects = []
        self.orientation_quats = []
        self.fingertipObjects = []

        self.text_buffer = []
        self.text_area = text(
            height=self.scale / 2,
            align='center',
            pos=vector(self.scale * 8, self.scale * 2, 0),
            color=vector(0,0,0),
            text="_"
        )

        for i in range(self.num):
            self.frontArrows.append(arrow(length=self.scale,shaftwidth=0.1 * self.scale,color=color.purple, axis=vector(1,0,0)))
            self.upArrows.append(arrow(length=self.scale,shaftwidth=0.1 * self.scale,color=color.magenta, axis=vector(0,1,0)))
            self.sideArrows.append(arrow(length=self.scale,shaftwidth=0.1 * self.scale,color=color.orange, axis=vector(0,0,1)))
            self.imuObjects.append(box(size=vector(0.2, 0.2, 0.2) * self.scale, pos=vector(0,0,0)))
            self.fingertipObjects.append(sphere(size=vector(0.2,0.2,0.2) * self.scale * 2, pos=vector(0,0,-1) * self.scale, opacity=0.5))
            self.orientation_quats.append(Quaternion(1, 0, 0, 0))
            front = self.orientation_quats[i].rotate([0.2,0,0])
            side = self.orientation_quats[i].rotate([0,0.2,0])
            up = self.orientation_quats[i].rotate([0,0,0.2])
            self.frontArrows[i].axis = vector(*front) * self.scale
            self.sideArrows[i].axis = vector(*side) * self.scale
            self.upArrows[i].axis = vector(*up) * self.scale
            self.imuObjects[i].axis = vector(*front) * self.scale
            self.imuObjects[i].up = vector(*up) * self.scale

    def put_key(self, key):
        if key == "space":
            self.text_buffer.append(" ")
        elif key == "bkspc":
            if len(self.text_buffer) > 0:
                self.text_buffer.pop()
        else:
            self.text_buffer.append(key)
        self.text_area.visible = False
        disp_text = "[ "+"".join(self.text_buffer)+" ]"
        self.text_area = text(
            height=self.scale / 2,
            align='center',
            pos=vector(self.scale * 8, self.scale * 2, 0),
            color=vector(0,0,0),
            text=disp_text
        )

    def display_keyboard(self, keymap: KeyMap):
        keys = keymap.key_order
        keywidths = keymap.key_widths
        ksize = keymap.key_size
        halfsize = ksize / 2
        for r, row in enumerate(keys):
            xpos = keymap.rows_offset[r]
            for c, key in enumerate(row):
                xpos += keywidths[r][c]
                box(
                    size=vector(ksize * keywidths[r][c], ksize, 0.001), 
                    pos=vector((xpos - keywidths[r][c] / 2) * ksize, -halfsize - r * ksize, 0),
                    color=vector(0.05 * c, 0.1 * r, 0.5)
                )
                text(
                    height=ksize / 3,
                    align='center',
                    pos=vector((xpos - keywidths[r][c] / 2) * ksize, -halfsize - r * ksize, 0.0005),
                    color=vector(1,1,1),
                    text=key
                )

    def register_debug_callback(self, callback):
        self.debug_callback = callback

    def register_step_callback(self, callback):
        self.step_callback = callback

    def register_reset_callback(self, callback):
        self.reset_callback = callback

    def _process_data(self, data: List[Data]) -> None:
        # Use vpython to view the sensor motion in space.

        try:
            keys = keysdown()
            if 'p' in keys:
                if not self.pdown and self.debug_callback:
                    self.debugging = self.debug_callback()
                self.pdown = True
            else:
                self.pdown = False
            if 'n' in keys:
                if not self.ndown and self.step_callback:
                    self.step_callback()
                self.ndown = True
            else:
                self.ndown = False
            if 'r' in keys:
                if not self.rdown and self.reset_callback:
                    self.reset_callback()
                self.rdown = True
            else:
                self.rdown = False
            for i in range(min(self.num, len(data))):
                orientation_quat = Quaternion(0,0,0,1) * data[i].orientation[-1]
                front = orientation_quat.rotate([-0.2,0,0])
                side = orientation_quat.rotate([0,-0.2,0])
                up = orientation_quat.rotate([0,0,0.1])
                self.frontArrows[i].axis = vector(*front) * self.scale
                self.sideArrows[i].axis = vector(*side) * self.scale
                self.upArrows[i].axis = vector(*up) * self.scale
                self.imuObjects[i].axis = vector(*front) * self.scale
                self.imuObjects[i].up = vector(*up) * self.scale

                # Negatives because IMUs mounted "upside down".
                imu_pos = vector(*[-data[i].final_position[-1][0], -data[i].final_position[-1][1], data[i].final_position[-1][2]]) + \
                            vector(*self.key_offsets[self.tracker_indices[i]])

                self.imuObjects[i].pos = imu_pos
                self.sideArrows[i].pos = imu_pos
                self.frontArrows[i].pos = imu_pos
                self.upArrows[i].pos = imu_pos

                finger_pos = vector(*[-data[i].finger_position[-1][0], -data[i].finger_position[-1][1], data[i].finger_position[-1][2]]) + \
                                vector(*self.key_offsets[self.tracker_indices[i]])

                self.fingertipObjects[i].pos = finger_pos

        except Exception as e:
            print("Output error:", e)
            raise e


class FileOutputStream(OutputStream):
    def __init__(self, file):
        super().__init__(100)
        self.file = file

    def put_metadata(self, tracker_indices, calibrations) -> None:
        with open(self.file, mode="w") as f:
            f.write("indices\n")
            f.write("accel cal x, accel cal y, accel cal z, gyro cal x, gyro cal y, gyro cal z\n")
            f.write(", ".join([f"{index}" for index in tracker_indices]) + "\n")
            f.write(" ".join([", ".join([f"{datapoint:.4f}" for datapoint in calibration]) for calibration in calibrations]) + "\n")
            f.write("time, raw accel x, raw accel y, raw accel z, final accel x, final accel y, final accel z, final velocity x, final velocity y, final velocity z, final position x, final position y, final position z, raw gyro roll, raw gyro pitch, raw gyro yaw, raw magnetometer x, raw mag y, raw mag z\n")

    def _process_data(self, data: List[Data]) -> None:
        with open(self.file, mode="a") as f:
            for i in range(-self._batchsize, 0):
                for j in range(len(data)):
                    f.write(", ".join([f"{data[j].time[i]:.4f}"] + [f"{datapoint:.4f}" for array in [data[j].raw_accel[i], data[j].final_accel[i], data[j].final_velocity[i], data[j].final_position[i], data[j].raw_gyro[i], data[j].raw_magnetometer[i]] for datapoint in array]))
                    f.write(" ")
                f.write("\n")


class GraphOutputStream(OutputStream):
    def __init__(self, index, num_disp_samples) -> None:
        super().__init__(15)
        self.tracker_data: Optional[Data] = None
        self.index = index
        self.num_disp_samples = num_disp_samples

        self.fig, self.ax = plt.subplots(nrows=4, ncols=1)

        self.acc_plot = self.ax[0]
        self.vel_plot = self.ax[1]
        self.pos_plot = self.ax[2]
        self.gyro_plot = self.ax[3]
        plt.style.use('seaborn-v0_8-whitegrid')
        
        plt.ion()
        plt.tight_layout()
        plt.pause(0.001)

    def _process_data(self, data: List[Data]) -> None:
        if self.index >= len(data):
            return

        self.tracker_data = data[self.index]

        if self.tracker_data is None:
            return

        self.acc_plot.cla()
        self.vel_plot.cla()
        self.pos_plot.cla()
        self.gyro_plot.cla()

        idx = self.tracker_data.time[-self.num_disp_samples:]             
        acc = self.tracker_data.final_accel[-self.num_disp_samples:] 
        vel = self.tracker_data.final_velocity[-self.num_disp_samples:] 
        pos = self.tracker_data.final_position[-self.num_disp_samples:]
        gyro = self.tracker_data.raw_gyro[-self.num_disp_samples:]
        
        self.acc_plot.plot(idx, acc)
        self.vel_plot.plot(idx, vel)
        self.pos_plot.plot(idx, pos)
        self.gyro_plot.plot(idx, gyro)
        
        plt.draw()
        plt.pause(0.001)

        plt.tight_layout()

