from typing import Tuple, List
import json
from touch_typer.util.utils import *
import math
import os

class KeyMap():

    def __init__(self, keymap_path = None) -> None:
        if keymap_path is None or len(keymap_path) == 0:
            keymap_path = os.path.abspath(os.path.join(__file__, os.path.pardir, "keymap.json"))

        with open(keymap_path, "r") as file:
            data= json.load(file)

        if 'key_size' in data:
            self.key_size = data['key_size']
        else:
            self.key_size = 1 # default size

        if 'rows_offset' in data:
           self.rows_offset = data['rows_offset']
        else: 
           self.rows_offset = [0, 0, 0, 0 ,0]
    
        if 'key_order' in data:
            self.key_order = data['key_order']
        else:
            raise Exception('key_order must be specified in the keymap json file')
    
        if 'starting_keys' in data:
            self.starting_keys = data['starting_keys']
        else:
            raise Exception('starting_keys must be specified in the keymap json file')

        if 'key_width_multipliers' in data:
            self.key_widths = data['key_width_multipliers']
        else:
            self.key_widths = [[1 for key in row] for row in key_order]

    def determine_key_by_coordinates(self, x, y, calib_key) -> str:
        row_index = min(max(0,     math.floor(y / self.key_size)),    len(self.key_order) - 1)

        col_index = 0
        col_loc = x / self.key_size - self.rows_offset[row_index]

        # All inputs to the left of keyboard become the leftmost key
        if col_loc < 0:
           col_loc = 0

        # All inputs to the right of keyboard become the rightmost key
        length_of_row = len(self.key_order[row_index])

        while col_loc > (w := self.key_widths[row_index][col_index]):
            col_index += 1
            if col_index >= length_of_row:
                col_index = length_of_row - 1
                break
            col_loc -= w

        return self.key_order[row_index][col_index]

    def determine_key_center_pos_tuple(self, key) -> tuple:
        
        def find_row_and_column(target):
            for row_index, row in enumerate(self.key_order):
                if target in row:
                    column_index = row.index(target)
                    return (row_index, column_index)

        # Get the center of the specified key on the imaginary keyboard
        y_key_idx, x_key_idx = find_row_and_column(key)

        x_key_pos = sum(self.key_widths[y_key_idx][:x_key_idx]) + self.rows_offset[y_key_idx] + 0.5 * self.key_widths[y_key_idx][x_key_idx]

        # Add 1/2 of keysize to each to find the actual center coordinates of the specific key
        x_key_idx += (0.5)
        y_key_pos = y_key_idx + (0.5)

        # Scale key positions to key size
        y_key_pos *= self.key_size
        x_key_pos *= self.key_size

        return x_key_pos, y_key_pos

