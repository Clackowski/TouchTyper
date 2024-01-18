import argparse
import keyboard
import numpy as np
from typing import List

from touch_typer.keypress.keymap import *
from touch_typer.keypress.keypress import *

from touch_typer.tracker.tracker import *
from touch_typer.tracker.filters import *
from touch_typer.tracker.input import *
from touch_typer.tracker.output import *
from touch_typer.util.quats import *
from touch_typer.util.utils import *
from threading import Thread


# Global variables for the main loop.
debug = False
need_step = True


def main():
    global debug, need_step

    # Use a nice command line argument parser.
    parser = argparse.ArgumentParser(
                    prog='tracker.py',
                    description='Runs a position estimation algorithm over the provided input.')
    parser.add_argument('-dl', '--device-left', default="", type=str)
    parser.add_argument('-dr', '--device-right', default="", type=str)
    parser.add_argument('-f', '--file', default="", type=str)
    parser.add_argument('-o', '--output-file', default="", type=str)
    parser.add_argument('-v', '--debug', action='store_true')
    parser.add_argument('-g', '--graph', default=-1, type=int)
    parser.add_argument('-bl', '--bluetooth-left', action='store_true')
    parser.add_argument('-br', '--bluetooth-right', action='store_true')
    parser.add_argument('-m', '--map', default="", type=str)

    args = parser.parse_args()

    # Set the debug variable appropriately.
    debug = args.debug

    # Load the keymap. #TODO could load another file here.
    keymap = KeyMap(args.map)

    finger_calibration_keys = keymap.starting_keys
    key_offsets = [np.array([*keymap.determine_key_center_pos_tuple(key), 0]) * [1,-1,1] for key in finger_calibration_keys]

    # Set up all the inputs. Can take inputs from devices connected over wires or BLE, or from files.
    input: Optional[InputStream] = None
    input_left: Optional[InputStream] = None
    input_right: Optional[InputStream] = None
    if args.file:
        input = FileInputStream(args.file)
        print("Using recorded data. Can modify filtering behavior in the code to test new results.")
    else: 
        if args.bluetooth_left:
            input_left = BluetoothInputStream(mac_address="ed:38:11:e0:13:7e")
            print("Connecting left hand over bluetooth.")
            inL = Thread(target=input_left.always_wait, args=[], daemon=True)
            inL.start()
        elif args.device_left:
            input_left = WiredArduinoInputStream(args.device_left)

        if args.bluetooth_right:
            input_right = BluetoothInputStream(mac_address="ad:da:93:fa:0c:68", right=True)
            print("Connecting right hand over bluetooth.")
            inR = Thread(target=input_right.always_wait, args=[], daemon=True)
            inR.start()
        elif args.device_right:
            input_right = WiredArduinoInputStream(args.device_right, right=True)

        if not input_left and not input_right:
            raise RuntimeError("No inputs specified.")
        
        if input_left and input_right:
            input = MergedInputStream(input_left, input_right)
        else:
            input = input_left if input_left else input_right

    # Figure out what trackers are actually connected
    # could have < 5 per hand, only one hand, only one tracker, etc.
    tracker_indices = input.get_tracker_indices()

    # JSON for sensor bias calibration.
    bias_path = os.path.abspath(os.path.join(__file__, os.path.pardir, "tracker/bias.json"))
    with open(bias_path, "r") as file:
        data = json.load(file)
    m = max(tracker_indices) + 1
    acc_bias = [[1, 1, 1, 0, 0, 0]] * m
    gyro_bias = [[0, 0, 0]] * m
    for i in tracker_indices:
        if 'tracker' + str(i) in data:
            tracker_bias_dict = data['tracker' + str(i)]
            acc_bias[i] = (tracker_bias_dict['acc'])
            gyro_bias[i] = (tracker_bias_dict['gyro'])

    # # Set up outputs.
    visual_output = VisualOutputStream(len(tracker_indices), keymap, key_offsets, tracker_indices)
    visual_output.register_debug_callback(toggle_debug)
    visual_output.register_step_callback(step)
    visual_output.register_reset_callback(reset)
    outputs: List[OutputStream] = [visual_output]

    # Get calibrations. TODO changeme to take in a chunk of time and create the calibration off of that.
    full_calibrations = input.get_calibration()
    calibrations = full_calibrations[1]
    frequency = full_calibrations[0]
    for output in outputs:
        output.put_metadata(tracker_indices, calibrations)
    
    # Filters that we are using for the trackers.
    acc_filter = lambda: SciPyLowPassOrder(6, 1/frequency, 6) # SciPyLowPass(10, 0.03)
    vel_filter = lambda: SciPyHighPassOrder(0.09, 1/frequency, 1)
    pos_filter = lambda: None # SciPyHighPassOrder(0.12, 1/frequency, 1)

    # Set up trackers with their initial positions
    trackers = [
        Tracker(acc_filter(), vel_filter(), pos_filter(), debug, f"tracker{i}", acc_bias[i], gyro_bias[i], keymap.key_size) 
            for i in tracker_indices
    ]


    # Debug stuff.
    def toggle_debug():
        global need_step
        global debug
        for t in trackers:
            t.toggle_debug()
        debug = not debug
        need_step = True
        return debug

    def step():
        global need_step
        need_step = True

    def reset():
        for t in trackers:
            t.reset()



    if args.output_file:
        file_output = FileOutputStream(args.output_file)
        outputs.append(file_output)

    if args.graph > -1:
        graph_output = GraphOutputStream(args.graph, 200) # should be 200 samples displayed at any time
        outputs.append(graph_output)



    # Start up the trackers.
    for i, tracker in enumerate(trackers):
        tracker.start(calibrations[i])
    
    # Set up keypress detectors
    keypress_detectors = [KeypressDetector(finger_calibration_keys[i], keymap) for i in tracker_indices]

    # TODO check if this solves our problem when we don't have the initial recalibration.
    #for keypress_detector in keypress_detectors:
        #keypress_detector.calibrate_initial_offsets()

    # Main loop
    iteration = 0
    while True:
        
        all_processed_data = []

        if need_step:
            new_data = None
            while new_data is None:
                new_data = input.get_new_data()

            need_input_reset = input.has_been_reset()
                
            for i, tracker in enumerate(trackers):
                # Primarily for file playback - resets inputs when necessary.
                if need_input_reset:
                    tracker.reset()

                # Update data with new tracked position, velocities, etc.
                override_dt = 0.03 if (debug or iteration == 0 or need_input_reset) else None
                processed_data = tracker.run_iteration(new_data[i], override_dt)

                all_processed_data.append(processed_data)
        
                # Put data for specific tracker into its corresponding keypressdetector to be processed
                keypress_detectors[i].put_new_data(processed_data)


            recalibration_stationary_samples = 40 # samples
            performing_recalibration = True

            if iteration > 0: # Always recalibrate on the first input
                for kp_detector in keypress_detectors:
                    performing_recalibration = performing_recalibration and (kp_detector.get_still_time() > recalibration_stationary_samples)

            for i, kp_detector in enumerate(keypress_detectors):
                pressed_key = kp_detector.is_key_pressed()

                # If key press is not none we print it, let the tracker know, and break loop since only one finger can be pressing a key at once
                if pressed_key:
                    x_press_loc, y_press_loc = keymap.determine_key_center_pos_tuple(pressed_key)
                    x_start_loc, y_start_loc = keymap.determine_key_center_pos_tuple(kp_detector.calibration_key)
                    tracker_reposition_offset = np.array([x_start_loc - x_press_loc, y_press_loc - y_start_loc, 0]) # TODO we might not care about the literal Z vals.
                    trackers[i].reset(finger_position=tracker_reposition_offset)
                    print(pressed_key, end='', flush=True) # TODO do operating system keypress output here instead!
                    visual_output.put_key(pressed_key)
                    try:
                        keyboard.press_and_release(pressed_key)
                    except Exception as err:
                        pass
                    break


            
            # TODO REALLY IMPORTANT - something happens when we move and tap somewhere and then recalibrate at that spot and then tap again, like a double offset.
            # Please figure that out and fix it please please pleas :)))



            # Once recalibration detector captures all data in a cycle
            # we can check if recalibration is happening, if so, alert
            # all of the trackers to reset themselves.
            if performing_recalibration:
                for kp_detector in keypress_detectors:
                    kp_detector.reset_still_time()

                for keypress_detector in keypress_detectors:
                    keypress_detector.calibrate_initial_offsets()

                for tracker in trackers:
                    new_calibration = create_calibration(tracker.data, recalibration_stationary_samples)
                    tracker.reset(calibration=new_calibration, finger_position=np.array([0,0,0]))

                #print('Recalibration complete for all fingers')


        if debug:
            need_step = False

        for output in outputs:
            output.put_new_data(all_processed_data)

        iteration += 1



if __name__ == "__main__":
    main()
