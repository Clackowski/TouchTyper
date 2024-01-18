import struct
import numpy as np
import serial
from typing import List, Optional
import time
import bluepy.btle as btle
from queue import Queue

FREQUENCY_HZ = 30
PERIOD_MS = 1000 / FREQUENCY_HZ


class InputStream():
    def get_calibration(self) -> Optional[np.ndarray]:
        # TODO maybe this shouldn't be the responsibility of the input stream
        # Take some samples while the sensor is stationary to be used as offsets.
        print("Calibrating...")
        end_time = time.time() + 2
        sampled_data = []
        first_time = None
        last_time = None
        while time.time() < end_time - 3:
            self.get_new_data()
        while time.time() < end_time:
            new_data = self.get_new_data()
            while (new_data is None):
                new_data = self.get_new_data()
            sampled_data.append(new_data[:,1:])
            if first_time is None:
                first_time = np.mean(np.array(new_data[:, 0]))    
            last_time = np.mean(np.array(new_data[:, 0]))
        frequency = len(sampled_data) / (last_time - first_time)
        mean_of_samples = np.mean(np.array(sampled_data), axis=0)
        self.accel_cal_vectors = mean_of_samples[:,0:3]
        self.gyro_cal_vectors = mean_of_samples[:,3:6]
        self.mag_cal_vectors = mean_of_samples[:,6:9]
        print(f"Accel calibrations are {self.accel_cal_vectors}")
        print(f"Gyro calibrations are {self.gyro_cal_vectors}")
        print("Can move the units now.")
        return (frequency, mean_of_samples)

    def get_tracker_indices(self) -> List[int]:
        new_data = self.get_new_data()
        while (new_data is None):
            new_data = self.get_new_data()
        return [(i + (5 if self.right else 0)) for i in range(len(new_data))]

    def get_new_data(self) -> Optional[np.ndarray]:
        """
        Gets new data for all sensors that correspond to this input.
        """
        pass

    def has_been_reset(self) -> bool:
        return False


class MergedInputStream(InputStream):
    def __init__(self, input0: InputStream, input1: InputStream) -> None:
        self.input0 = input0
        self.input1 = input1
        self.new_data0 = None
        self.new_data1 = None

    def get_tracker_indices(self) -> List[int]:
        return self.input0.get_tracker_indices() + self.input1.get_tracker_indices()

    def get_new_data(self) -> Optional[np.ndarray]:
        if self.new_data0 is None:
            self.new_data0 = self.input0.get_new_data()
        if self.new_data1 is None:
            self.new_data1 = self.input1.get_new_data()
        if self.new_data0 is not None and self.new_data1 is not None:
            ret = np.append(self.new_data0, self.new_data1, axis=0)
            self.new_data0 = None
            self.new_data1 = None
            return ret


class WiredArduinoInputStream(InputStream):
    def __init__(self, device: str, right: bool = False):
        self.right = right
        self.connection = serial.Serial(device, 115200)
        time.sleep(1)

    def get_new_data(self) -> Optional[np.ndarray]:
        # Receive new data from the Arduino. Propagates errors.
        while self.connection.in_waiting == 0:
            pass
        try:
            new_data = self.connection.readline().strip().decode().replace(","," ").split()
            t = new_data[0] # timestamp is the first data point
            new_data = np.reshape(np.array(new_data[1:]).astype(np.float32), [-1, 9]) # 9 data points per unit (accel, gyro, mag)
            new_data = np.c_[[t] * len(new_data), new_data].astype(np.float32) # Prepend time to every row of data, for convenience
            return new_data
        except:
            return None


class FileInputStream(InputStream):
    def __init__(self, file: str):
        with open(file) as f:
            f.readline()
            f.readline()
            self.tracker_indices = [int(index) for index in f.readline().strip().replace(",", " ").split()]
            self.calibration = np.array([float(datapoint) for datapoint in f.readline().strip().replace(",", " ").split()]).reshape([-1, 9])
            f.readline()
            self.lines = [line.strip() for line in f.readlines()]
        self.iter = 0
        self.num_lines = len(self.lines)

    def get_tracker_indices(self) -> List[int]:
        return self.tracker_indices

    def get_calibration(self) -> Optional[np.ndarray]:
        return self.calibration

    def get_new_data(self) -> Optional[np.ndarray]:
        # Read the next line in the file, throw an error if out of input.
        split_line = self.lines[self.iter].replace(",", " ").split()
        self.iter = (self.iter + 1) % self.num_lines
        # Repeatedly grab time, raw accel, gyro, mag
        new_data = []
        while len(split_line) > 0:
            new_data += split_line[0:1] + split_line[1:4] + split_line[13:16] + split_line[16:19]
            split_line = split_line[19:]
        new_data = np.reshape(np.array(new_data).astype(np.float32), [-1, 10]) # 10 data points per unit (time, accel, gyro, mag)
        time.sleep(PERIOD_MS / 1000) # TODO sleep by the delta t instead
        return new_data

    def has_been_reset(self) -> bool:
        if self.iter == 0:
            print("END OF RECORDED DATA; LOOPING")
        return self.iter == 0


class BluetoothInputStream(InputStream):

    def __init__(self, mac_address: str, right: bool = False) -> None:
        self.right = right

        # Find the peripheral nd service
        print('Attemping to connect bluetooth with device of MAC address: ' + str(mac_address))
        self.peripheral = btle.Peripheral(mac_address)
        self.peripheral.setMTU(200)
        self.services = self.peripheral.getServices()
        self.service = self.peripheral.getServiceByUUID(list(self.services)[2].uuid)
        print('Found Service')
        self.all_characteristics = self.service.getCharacteristics()
        print('Found ' + str(len(self.all_characteristics)) + ' characteristics.')

        # Get and subscribe to the data characteristic
        dataCharacteristic = self.all_characteristics[0]
        self.peripheral.writeCharacteristic(dataCharacteristic.valHandle + 1, b'\x01\x00')
        self.handle = dataCharacteristic.getHandle()

        # Read number of imus from the arduino
        num_imus_cx = self.all_characteristics[1].read()
        self.numImus = struct.unpack('i', num_imus_cx)[0]

        self.dataqueue = Queue() #do we want a stack here? Only pay attention to the latest, I could very much see us getting farther and farther behind if injected when not waiting

        self.peripheral.setDelegate(NotifyReceiver(self))
        self.wait = True
    
    # Temporary, for testing purposes
    def get_tracker_indices(self) -> List[int]:
        if self.right:
            return [5, 6, 7, 8, 9]
        return [0, 1, 2, 3, 4]

    def get_new_data(self) -> Optional[np.ndarray]:
        if not self.dataqueue.empty():
            popped_data = self.dataqueue.get()
            return popped_data
        return None
        # self.peripheral.waitForNotifications(1.0)

        # if self.dataqueue.empty():
        #     print('empty dataqueue')
        #     return None
        # else:
        #     popped_data = self.dataqueue.get()
        #     return popped_data
    
    def put_new_data(self, new_data):
        self.dataqueue.put(new_data)

    def get_num_imus(self):
        return self.numImus
    
    def always_wait(self):
        while self.wait:
            self.peripheral.waitForNotifications(1.0)
    
    def stop_waiting(self):
        self.wait = False

    def __del__(self):
        # Destructor disconnects bluetooth connection
        #self.p.disconnect()
        pass

class NotifyReceiver(btle.DefaultDelegate):

    def __init__(self, stream: BluetoothInputStream):
        btle.DefaultDelegate.__init__(self)
        self.stream = stream
        self.numImus = stream.get_num_imus()
    
    def handleNotification(self, cHandle, data):

        if cHandle == self.stream.handle:
            # There will be 9 floats for every imu connected, we also use 1 Long to send the time
            destructure_format = '=L' + ('f' * 9) * self.numImus
            new_data = list(struct.unpack(destructure_format, data))
            t = new_data[0]
            # print(t)
            new_data = np.reshape(np.array(new_data[1:]).astype(np.float32), [-1, 9])
            new_data = np.c_[[t] * len(new_data), new_data] # Prepend time to every row of data, for convenience
            self.stream.put_new_data(new_data)
