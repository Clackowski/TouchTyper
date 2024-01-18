from queue import *
from typing import Tuple, List
import json
from touch_typer.util.utils import *
from touch_typer.keypress.keymap import *


class KeypressDetector():

    # needs to init with some sort of keymap
    def __init__(self, calibration_key, keymap) -> None:

        # This is the key we are going to assume finger is on when position is calibrated
        self.calibration_key = calibration_key

        # We can decide to not consider the Z position of a key tap when deciding if it counts as a press, and just rely on tapping motion
        self.ignore_finger_depth = True # TODO maybe revert, maybe put as constructor arg

        # This keeps track of how long this keypress detector has been still
        self.still_time = 0

        self.keymap = keymap

        self.x_calib_offset = 0
        self.y_calib_offset = 0
        self.z_calib_offset = 0

        self.most_recent_data = {}
        self.recent_data_queue = Queue()


    # Add most recent data to the recent data queue
    def update_queue(self) -> None:
        if self.recent_data_queue.qsize() > 10:
            self.recent_data_queue.get(0)

        self.recent_data_queue.put(self.most_recent_data)


    def put_new_data(self, data: Data) -> None:
        # Update most recent data, reversing the Y data to match the keyboard structure
        self.most_recent_data = data.to_dict_with_most_recent_data(True)
        # Update the queue
        self.update_queue()
        
        # Update still_time based on velocity changes

        if self.did_finger_move():
            # Reset still_time since finger has moved since last timestep
            self.still_time = 0 
        else:
            # Increment still_time since the finger has been still for another timestep
            self.still_time += 1


    def is_key_pressed(self):
        # Returns str of key if exists, otherwise returns none

        # First call is_tap_motion to see if there is downward accel
        if self.is_tap_motion(self.most_recent_data):

            # Determine whether a key is pressed or not, and if so what key
            
            # Check if z height is low enough that finger is actually touching the key
            z_pos = self.most_recent_data['z_pos']

            # Determine whether or not the z position is low enough to be considered touching the keyboard
            finger_contact_with_key = (abs(self.z_calib_offset - z_pos) < 0.004) or self.ignore_finger_depth # 4 millimeters from key

            if finger_contact_with_key:

                # Extract current x and y positions from the data
                x_pos = self.most_recent_data['x_pos']
                y_pos = self.most_recent_data['y_pos']

                # print("x calib offset", self.x_calib_offset)
                # print("x pos         ", x_pos)
                # print("y calib offset", self.y_calib_offset)
                # print("y pos         ", y_pos)

                # use KeyMap method to check which key is pressed based on positon data
                return self.keymap.determine_key_by_coordinates(self.x_calib_offset + x_pos, self.y_calib_offset + y_pos, self.calibration_key)

            # No Finger contact with key = no keypress
            else:
                return None

        # No tap = no keypress
        else: 
            return None

    def calibrate_initial_offsets(self) -> None:
        # Set initial positions of keypress detectors to the middles of the following keys
        #   asdf  jkl;
        x_pos = self.most_recent_data['x_pos']
        y_pos = self.most_recent_data['y_pos']
        z_pos = self.most_recent_data['z_pos']

        # Get the center of the specified key on the imaginary keyboard
        x_key_center, y_key_center = self.keymap.determine_key_center_pos_tuple(self.calibration_key)

        # Difference of current x y offsets to the key centers
        self.x_calib_offset = x_key_center # - x_pos # XXX(ndorson): don't need these, that portion of reset happens in the tracker
        self.y_calib_offset = y_key_center # - y_pos
        # We can now use these offset vectors to determine which key is pressed

        # Calibrate Z to fingers resting on keyboard
        self.z_calib_offset = 0 # z_pos

    def get_positions_tuple(self):
        return (self.x_calib_offset, self.y_calib_offset, self.z_calib_offset)

    def get_still_time(self):
        return self.still_time

    def reset_still_time(self):
        self.still_time = 0

    def did_finger_move(self) -> bool:
        # Get x, y, z velocities
        x_vel = self.most_recent_data['x_vel']
        y_vel = self.most_recent_data['y_vel']
        z_vel = self.most_recent_data['z_vel']

        x_threshold = 0.01 # Change later (m/s)
        y_threshold = 0.01 # Change later (m/s)
        z_threshold = 0.01 # Change later (m/s)

        #return abs(x_vel) > x_threshold or abs(y_vel) > y_threshold or abs(z_vel) > z_threshold

        x_acc = self.most_recent_data['x_acc']
        y_acc = self.most_recent_data['y_acc']
        z_acc = self.most_recent_data['z_acc']
        thresh = 0.1

        return abs(x_acc) > thresh or abs(y_acc) > thresh or abs(z_vel) > thresh

    def is_tap_motion(self, data: Data) -> bool:
        acc_threshold = -2 # Change later

        # Convert queue into list
        data_queue_as_list = list(self.recent_data_queue.queue)

        # Create list of z accelerations and list of z velocities
        # TODO this may be inefficient, may not matter at all though.
        list_of_accels = []
        list_of_vels = []
        for data in data_queue_as_list:
            list_of_accels.append(data['z_acc'])
            list_of_vels.append(data['z_vel'])

        # Check if all accelerations are above a threshold
        all_acc_below_thresh = all(acc <= acc_threshold for acc in list_of_accels[-3:]) # TODO no magic plz

        #Check if all velocities are negative or 0
        all_vel_negative = all(vel <= 0 for vel in list_of_vels[-3:]) #TODO no magic plz

        # Print to terminal if tap motion occurs
        if all_acc_below_thresh and all_vel_negative:
            pass # print("Tap motion occurred")

        # Check all conditions to see if tapping
        return (all_acc_below_thresh and all_vel_negative)

