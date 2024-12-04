#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Combined Script for MPU6050 and XM125 Sensors
Optimized for High Performance, Low Latency, High Throughput, Concurrency, and Extensibility
"""

import smbus2
import RPi.GPIO as GPIO
import time
import math
import struct
from enum import IntEnum
import threading
import queue
import logging
import board
import adafruit_hcsr04
import json

# ============================
# Constants and Configuration
# ============================

# I2C Configuration
I2C_BUS = 1  # Typically, I2C bus 1 on Raspberry Pi 4
MPU6050_ADDR = 0x68
XM125_ADDR = 0x52

# MPU6050 Register Addresses
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# XM125 Register Addresses and Configuration
REGISTER_MAP = {
    # Read Only
    0x0000: 'Version',
    0x0001: 'Protocol Status',
    0x0002: 'Measure Counter',
    0x0003: 'Detector Status',
    0x0010: 'Distance Result',
    0x0011: 'Peak0 Distance',
    0x0012: 'Peak1 Distance',
    0x0013: 'Peak2 Distance',
    0x0014: 'Peak3 Distance',
    0x0015: 'Peak4 Distance',
    0x0016: 'Peak5 Distance',
    0x0017: 'Peak6 Distance',
    0x0018: 'Peak7 Distance',
    0x0019: 'Peak8 Distance',
    0x001A: 'Peak9 Distance',
    0x001B: 'Peak0 Strength',
    0x001C: 'Peak1 Strength',
    0x001D: 'Peak2 Strength',
    0x001E: 'Peak3 Strength',
    0x001F: 'Peak4 Strength',
    0x0020: 'Peak5 Strength',
    0x0021: 'Peak6 Strength',
    0x0022: 'Peak7 Strength',
    0x0023: 'Peak8 Strength',
    0x0024: 'Peak9 Strength',
    0xFFFF: 'Application Id',

    # Read / Write
    0x0040: 'Start',
    0x0041: 'End',
    0x0042: 'Max Step Length',
    0x0043: 'Close Range Leakage Cancellation',
    0x0044: 'Signal Quality',
    0x0045: 'Max Profile',
    0x0046: 'Threshold Method',
    0x0047: 'Peak Sorting',
    0x0048: 'Num Frames Recorded Threshold',
    0x0049: 'Fixed Amplitude Threshold Value',
    0x004A: 'Threshold Sensitivity',
    0x004B: 'Reflector Shape',
    0x004C: 'Fixed Strength Threshold Value',
    0x0080: 'Measure On Wakeup',

    # Write Only
    0x0100: 'Command',
}

# Command Values
COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2
COMMAND_APPLY_CONFIG = 3
COMMAND_CALIBRATE = 4
COMMAND_RECALIBRATE = 5
COMMAND_ENABLE_UART_LOGS = 32
COMMAND_DISABLE_UART_LOGS = 33
COMMAND_LOG_CONFIGURATION = 34
COMMAND_RESET_MODULE = 1381192737  # 0x52535421

# Data file path
DATA_FILE = 'sensor_data.jsonl'

# GPIO Pin Configuration
# MPU6050 does not require additional GPIO pins
XM125_WAKE_UP_PIN = 17  # GPIO17 (Physical pin 11)
XM125_INT_PIN = 27       # GPIO27 (Physical pin 13)

# HC-SR04 GPIO Pins
HC_SR04_TRIG = board.D23  # 使用board库的D23引脚
HC_SR04_ECHO = board.D24  # 使用board库的D24引脚
HC_SR04_INTERVAL = 0.1    # 每0.1秒测量一次

# Measurement Intervals
MPU6050_INTERVAL = 0.1   # 100 milliseconds
XM125_INTERVAL = 0.01     # 100 milliseconds

# ============================
# Enumerations
# ============================

class ThresholdMethod(IntEnum):
    FIXED_AMPLITUDE = 1
    RECORDED = 2
    CFAR = 3
    FIXED_STRENGTH = 4

class PeakSortingMethod(IntEnum):
    CLOSEST = 1
    STRONGEST = 2

class ReflectorShape(IntEnum):
    GENERIC = 1
    PLANAR = 2

class Profile(IntEnum):
    PROFILE1 = 1
    PROFILE2 = 2
    PROFILE3 = 3
    PROFILE4 = 4
    PROFILE5 = 5

# ============================
# Detector Configuration Parameters
# ============================

DetectorConfig = {
    'start_m': 0.1,  # meters (10 cm)
    'end_m': 3.0,     # meters (3 m)
    'max_step_length': 0,  # 0 means auto
    'max_profile': Profile.PROFILE1,
    'close_range_leakage_cancellation': False,  # Set to True if measuring close to sensor (<10 cm)
    'signal_quality': 30.0,  # dB
    'threshold_method': ThresholdMethod.CFAR,
    'peaksorting_method': PeakSortingMethod.CLOSEST,
    'reflector_shape': ReflectorShape.GENERIC,
    'num_frames_in_recorded_threshold': 100,
    'fixed_threshold_value': 100.0,  # factor 1000 larger
    'fixed_strength_threshold_value': 0.0,  # factor 1000 larger
    'threshold_sensitivity': 500,  # factor 1000 larger
    'measure_on_wakeup': False,  # Prevent KeyError
}

# ============================
# I2C and GPIO Initialization
# ============================

# Configure logging to file, overwrite each run
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    filename='sensor.log',  # Log file name
    filemode='w'  # Overwrite mode
)

# Disable GPIO warnings
GPIO.setwarnings(False)

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Initialize XM125 GPIO pins
GPIO.setup(XM125_WAKE_UP_PIN, GPIO.OUT)
GPIO.setup(XM125_INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize I2C bus
bus = smbus2.SMBus(I2C_BUS)

# Create a lock to synchronize I2C bus access
i2c_lock = threading.Lock()
# ============================
# HC_SR04 Class
# ============================

class HC_SR04_Sensor:
    def __init__(self, trig_pin, echo_pin, data_queue=None):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.data_queue = data_queue  # 用于将数据传递到主线程
        self.running = True
        self.sonar = adafruit_hcsr04.HCSR04(trigger_pin=self.trig_pin, echo_pin=self.echo_pin)
    
    def run(self):
        """
        运行HC-SR04的测量循环。
        """
        logging.info("HC-SR04 measurement thread started.")
        while self.running:
            try:
                distance = self.sonar.distance  # 获取距离，单位为厘米
                distance_m = distance / 100.0    # 转换为米
                data = {
                    'type': 'HC_SR04',
                    'distance_m': distance_m,
                    'timestamp': time.time()
                }
                if self.data_queue and distance is not None:
                    self.data_queue.put(data)
                time.sleep(HC_SR04_INTERVAL)
            except RuntimeError:
                logging.warning("HC-SR04 measurement failed. Retrying!")
            except Exception as e:
                logging.error(f"Exception in HC-SR04 run loop: {e}")
                self.running = False
        logging.info("HC-SR04 measurement thread terminated.")
    
    def stop(self):
        self.running = False

# ============================
# MPU6050 Class
# ============================

class MPU6050:
    def __init__(self, bus, address=MPU6050_ADDR, data_queue=None):
        self.bus = bus
        self.address = address
        self.data_queue = data_queue  # Used to pass data to the main thread
        self.running = True

    def initialize(self):
        # Wake up the device
        with i2c_lock:
            self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        logging.info("MPU6050 initialized and awakened.")

    def read_raw_data(self, reg):
        with i2c_lock:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def read_accelerometer(self):
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = self.read_raw_data(ACCEL_XOUT_H + 4)
        # Default ±2g mode, convert raw values to g
        return acc_x * 0.000061, acc_y * 0.000061, acc_z * 0.000061

    def read_gyroscope(self):
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
        # Default ±250°/s mode, convert raw values to °/s
        return gyro_x * 0.00763, gyro_y * 0.00763, gyro_z * 0.00763

    def run(self):
        # Initialize previous accelerometer and gyroscope readings
        prev_acc = self.read_accelerometer()
        prev_gyro = self.read_gyroscope()

        while self.running:
            try:
                # Current accelerometer and gyroscope readings
                current_acc = self.read_accelerometer()
                current_gyro = self.read_gyroscope()

                # Calculate change magnitude
                acc_dx, acc_dy, acc_dz, acc_total = calculate_change(prev_acc, current_acc)
                gyro_dx, gyro_dy, gyro_dz, gyro_total = calculate_change(prev_gyro, current_gyro)

                # Construct data dictionary
                data = {
                    'type': 'MPU6050',
                    'acc_change': {
                        'dX': acc_dx,
                        'dY': acc_dy,
                        'dZ': acc_dz,
                        'Total Change': acc_total
                    },
                    'gyro_change': {
                        'dX': gyro_dx,
                        'dY': gyro_dy,
                        'dZ': gyro_dz,
                        'Total Change': gyro_total
                    },
                    'timestamp': time.time()
                }

                # Put data into the queue
                if self.data_queue:
                    self.data_queue.put(data)

                # Update previous readings
                prev_acc = current_acc
                prev_gyro = current_gyro

                time.sleep(MPU6050_INTERVAL)
            except Exception as e:
                logging.error(f"Exception in MPU6050 run loop: {e}")
                self.running = False

    def stop(self):
        self.running = False

# ============================
# XM125 Class
# ============================

class XM125:
    def __init__(self, bus, wake_up_pin, int_pin, address=XM125_ADDR, data_queue=None):
        self.bus = bus
        self.address = address
        self.wake_up_pin = wake_up_pin
        self.int_pin = int_pin
        self.data_queue = data_queue  # Used to pass data to the main thread
        self.running = True

    def scan_i2c(self):
        """
        Scans the I2C bus for connected devices.
        """
        devices = []
        logging.info("Scanning I2C bus for devices...")
        for address in range(0x03, 0x78):
            try:
                with i2c_lock:
                    self.bus.write_quick(address)
                devices.append(address)
            except Exception:
                pass  # Device did not acknowledge
        if devices:
            logging.info(f"Found device(s) at address(es): {', '.join(['0x%02X' % addr for addr in devices])}")
        else:
            logging.warning("No I2C devices found.")
        return devices

    def read_register(self, reg_addr, signed=False):
        """
        Reads a 32-bit register from the XM125 module.
        """
        try:
            reg_bytes = struct.pack('>H', reg_addr)
            read = smbus2.i2c_msg.read(self.address, 4)
            write = smbus2.i2c_msg.write(self.address, reg_bytes)
            with i2c_lock:
                self.bus.i2c_rdwr(write, read)
            data = bytes(read)
            if len(data) != 4:
                logging.error(f"Read error: Expected 4 bytes, got {len(data)} bytes")
                return None
            value = struct.unpack('>I', data)[0] if not signed else struct.unpack('>i', data)[0]
            return value
        except Exception as e:
            logging.error(f"Exception while reading register 0x{reg_addr:04X}: {e}")
            return None

    def write_register(self, reg_addr, value):
        """
        Writes a 32-bit value to a register in the XM125 module.
        """
        try:
            reg_bytes = struct.pack('>H', reg_addr)
            value_bytes = struct.pack('>I', value)
            data = reg_bytes + value_bytes
            write = smbus2.i2c_msg.write(self.address, data)
            with i2c_lock:
                self.bus.i2c_rdwr(write)
            return True
        except Exception as e:
            logging.error(f"Exception while writing to register 0x{reg_addr:04X}: {e}")
            return False

    def wait_for_int(self, timeout=10):
        """
        Waits for the INT pin to go HIGH indicating the module is ready.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if GPIO.input(self.int_pin) == GPIO.HIGH:
                return True
            time.sleep(XM125_INTERVAL)
        logging.warning("Timeout waiting for INT pin.")
        return False

    def initialize_module(self):
        """
        Initializes the XM125 module by waking it up and ensuring it's ready.
        """
        # Set WAKE_UP high
        GPIO.output(self.wake_up_pin, GPIO.HIGH)
        logging.info("XM125 WAKE_UP set to HIGH.")

        # Wait for INT pin to go HIGH
        if not self.wait_for_int():
            logging.error("XM125 module did not become ready.")
            return False
        logging.info("XM125 module is ready for I2C communication.")
        return True

    def write_default_configuration(self):
        """
        Writes the default configuration registers to the XM125 module.
        """
        config_registers = {
            0x0040: int(DetectorConfig['start_m'] * 1000),  # Start (mm)
            0x0041: int(DetectorConfig['end_m'] * 1000),    # End (mm)
            0x0042: DetectorConfig['max_step_length'],      # Max Step Length
            0x0043: int(DetectorConfig['close_range_leakage_cancellation']),  # Close Range Leakage Cancellation (bool)
            0x0044: int(DetectorConfig['signal_quality'] * 1000),  # Signal Quality (dB * 1000)
            0x0045: DetectorConfig['max_profile'],          # Max Profile
            0x0046: DetectorConfig['threshold_method'],     # Threshold Method
            0x0047: DetectorConfig['peaksorting_method'],   # Peak Sorting
            0x0048: DetectorConfig['num_frames_in_recorded_threshold'],  # Num Frames Recorded Threshold
            0x0049: int(DetectorConfig['fixed_threshold_value'] * 1000),  # Fixed Amplitude Threshold Value
            0x004A: DetectorConfig['threshold_sensitivity'],             # Threshold Sensitivity
            0x004B: DetectorConfig['reflector_shape'],                   # Reflector Shape
            0x004C: int(DetectorConfig['fixed_strength_threshold_value'] * 1000),  # Fixed Strength Threshold Value
            0x0080: int(DetectorConfig['measure_on_wakeup']),            # Measure On Wakeup (bool)
        }

        logging.info("Writing default configuration to XM125...")
        for reg, val in config_registers.items():
            success = self.write_register(reg, val)
            if not success:
                logging.error(f"Failed to write to register 0x{reg:04X}.")
                return False
            logging.info(f"Register 0x{reg:04X} set to {val}.")
        return True

    def apply_config_and_calibrate(self):
        """
        Applies configuration settings and calibrates the XM125 module.
        """
        if not self.write_default_configuration():
            logging.error("Failed to write default configuration.")
            return False

        # Send APPLY_CONFIG_AND_CALIBRATE command
        if not self.write_register(0x0100, COMMAND_APPLY_CONFIG_AND_CALIBRATE):  # COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
            logging.error("Failed to send APPLY_CONFIG_AND_CALIBRATE command.")
            return False
        logging.info("APPLY_CONFIG_AND_CALIBRATE command sent.")

        # Wait for BUSY bit to clear
        logging.info("Waiting for XM125 to finish configuration and calibration...")
        while True:
            status = self.read_register(0x0003)  # Detector Status
            if status is None:
                logging.error("Failed to read Detector Status.")
                return False
            if not (status & 0x80000000):  # BUSY mask
                break
            time.sleep(XM125_INTERVAL)
        logging.info("Configuration and calibration completed.")

        # Check for Detector Errors
        error_flags = (status >> 16) & 0xFF  # Bits 16 to 23
        if error_flags != 0:
            logging.error(f"Detector Status Error Flags: 0x{error_flags:02X}")
            return False
        return True

    def perform_recalibration(self):
        """
        Performs recalibration by sending the RECALIBRATE command.
        """
        # Send RECALIBRATE command
        if not self.write_register(0x0100, COMMAND_RECALIBRATE):  # COMMAND_RECALIBRATE = 5
            logging.error("Failed to send RECALIBRATE command.")
            return False
        logging.info("RECALIBRATE command sent.")

        # Wait for BUSY bit to clear
        logging.info("Waiting for XM125 to finish recalibration...")
        while True:
            status = self.read_register(0x0003)  # Detector Status
            if status is None:
                logging.error("Failed to read Detector Status during recalibration.")
                return False
            if not (status & 0x80000000):  # BUSY mask
                break
            time.sleep(XM125_INTERVAL)
        logging.info("Recalibration completed.")

        # Check for Detector Errors
        error_flags = (status >> 16) & 0xFF  # Bits 16 to 23
        if error_flags != 0:
            logging.error(f"Detector Status Error Flags during recalibration: 0x{error_flags:02X}")
            return False
        return True

    def measure_distance(self):
        """
        Triggers a distance measurement and reads the result.
        """
        # Send MEASURE_DISTANCE command
        if not self.write_register(0x0100, COMMAND_MEASURE_DISTANCE):  # COMMAND_MEASURE_DISTANCE = 2
            logging.error("Failed to send MEASURE_DISTANCE command.")
            return (None, None)
        logging.info("MEASURE_DISTANCE command sent.")

        # Wait for BUSY bit to clear
        logging.info("Waiting for XM125 to complete measurement...")
        while True:
            status = self.read_register(0x0003)  # Detector Status
            if status is None:
                logging.error("Failed to read Detector Status.")
                return (None, None)
            if not (status & 0x80000000):  # BUSY mask
                break
            time.sleep(XM125_INTERVAL)
        logging.info("Measurement completed.")

        # Read Distance Result
        distance_result = self.read_register(0x0010)
        if distance_result is None:
            logging.error("Failed to read Distance Result.")
            return (None, None)

        # Parse Distance Result
        num_distances = distance_result & 0x0000000F
        near_start_edge = (distance_result >> 8) & 0x1
        calibration_needed = (distance_result >> 9) & 0x1
        measure_distance_error = (distance_result >> 10) & 0x1
        temperature = (distance_result >> 16) & 0xFFFF

        if measure_distance_error:
            logging.error("Measurement failed (MEASURE_DISTANCE_ERROR).")
            return (None, None)

        if calibration_needed:
            logging.warning("Calibration needed. Initiating recalibration...")
            if not self.perform_recalibration():
                logging.error("Recalibration failed.")
                return (None, None)
            logging.info("Recalibration successful. Please retry the measurement.")
            return (None, None)  # Skip this measurement

        if num_distances == 0:
            logging.info("No peaks detected.")
            return ([], [])

        # Ensure num_distances does not exceed 10
        num_distances = min(num_distances, 10)
        logging.info(f"Number of distances detected: {num_distances}")

        # Read Peaks
        distances_m = []
        strengths_db = []
        for i in range(num_distances):
            peak_dist_addr = 0x0011 + i  # 0x0011 to 0x001A
            peak_str_addr = 0x001B + i   # 0x001B to 0x0024

            distance_raw = self.read_register(peak_dist_addr)
            strength_raw = self.read_register(peak_str_addr, signed=True)

            if distance_raw is not None and strength_raw is not None:
                # Convert to meters and dB
                distance_m = distance_raw / 1000.0  # mm to meters
                strength_db_val = strength_raw / 1000.0  # dB

                distances_m.append(distance_m)
                strengths_db.append(strength_db_val)
            else:
                logging.error(f"Failed to read Peak{i} data.")

        return (distances_m, strengths_db)

    def run(self):
        """
        Runs the main loop for the XM125 sensor.
        """
        try:
            # Scan I2C bus to confirm device exists
            devices = self.scan_i2c()
            if self.address not in devices:
                logging.error(f"XM125 device with address 0x{self.address:02X} not found on the I2C bus.")
                return
            else:
                logging.info(f"Found XM125 device at address 0x{self.address:02X} on the I2C bus.")

            # Initialize module
            logging.info("Initializing XM125 module...")
            if not self.initialize_module():
                logging.error("XM125 module initialization failed.")
                return

            # Apply configuration and calibrate
            logging.info("Applying configuration and calibrating XM125 module...")
            if not self.apply_config_and_calibrate():
                logging.error("Applying configuration and calibrating XM125 module failed.")
                return

            logging.info("Entering measurement loop for XM125.")
            while self.running:
                distances, strengths = self.measure_distance()
                # Construct data dictionary
                data = {
                    'type': 'XM125',
                    'distances_m': distances,
                    'strengths_db': strengths,  # Correct variable name
                    'timestamp': time.time()
                }
                if self.data_queue and distances is not None and strengths is not None:
                    self.data_queue.put(data)
                time.sleep(XM125_INTERVAL)
        except Exception as e:
            logging.error(f"Exception in XM125 run loop: {e}")
        finally:
            logging.info("XM125 thread has terminated.")

    def stop(self):
        self.running = False

# ============================
# Helper Functions
# ============================

def calculate_change(prev, current):
    """
    Calculates the magnitude of changes across three axes.
    :param prev: Previous reading (x, y, z)
    :param current: Current reading (x, y, z)
    :return: Change in each axis, total change magnitude
    """
    dx = current[0] - prev[0]
    dy = current[1] - prev[1]
    dz = current[2] - prev[2]
    total_change = math.sqrt(dx**2 + dy**2 + dz**2)  # Euclidean distance
    return dx, dy, dz, total_change

# ============================
# Main Function
# ============================

def main():
    # Create a data queue
    data_queue = queue.Queue()
    
    # Initialize MPU6050
    mpu = MPU6050(bus, data_queue=data_queue)
    mpu.initialize()

    # Initialize XM125
    xm125 = XM125(bus, XM125_WAKE_UP_PIN, XM125_INT_PIN, data_queue=data_queue)

    # Initialize HC_SR04
    hc_sr04 = HC_SR04_Sensor(HC_SR04_TRIG, HC_SR04_ECHO, data_queue=data_queue)

    # Create threads
    mpu_thread = threading.Thread(target=mpu.run, name="MPU6050-Thread")
    xm125_thread = threading.Thread(target=xm125.run, name="XM125-Thread")
    hc_sr04_thread = threading.Thread(target=hc_sr04.run, name="HC_SR04-Thread")

    # Start threads
    mpu_thread.start()
    xm125_thread.start()
    hc_sr04_thread.start()

    # Open data file in write mode (overwrite each run)
    with open(DATA_FILE, 'w') as data_file:
        try:
            while True:
                try:
                    data = data_queue.get(timeout=1)  # Block and wait for data
                    if data['type'] == 'MPU6050':
                        acc = data['acc_change']
                        gyro = data['gyro_change']
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data['timestamp']))
                        logging.info(f"MPU6050 Acceleration Change: dX={acc['dX']:.4f} g, dY={acc['dY']:.4f} g, dZ={acc['dZ']:.4f} g, Total Change={acc['Total Change']:.4f} g")
                        logging.info(f"MPU6050 Gyroscope Change: dX={gyro['dX']:.4f} °/s, dY={gyro['dY']:.4f} °/s, dZ={gyro['dZ']:.4f} °/s, Total Change={gyro['Total Change']:.4f} °/s")
                    elif data['type'] == 'XM125':
                        distances = data['distances_m']
                        strengths = data['strengths_db']
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data['timestamp']))
                        if distances and strengths:
                            for idx, (dist, stren) in enumerate(zip(distances, strengths)):
                                logging.info(f"XM125 Peak{idx}: Distance = {dist:.3f} m | Strength = {stren:.3f} dB")
                        else:
                            logging.info("XM125 No peaks detected or measurement failed.")
                    elif data['type'] == 'HC_SR04':
                        distance = data['distance_m']
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data['timestamp']))
                        if distance is not None:
                            logging.info(f"HC-SR04 Distance Measurement: {distance:.2f} meters")
                        else:
                            logging.info("HC-SR04 Distance Measurement Failed or Timed Out.")
                    
                    # Write data to the data file as JSON
                    json.dump(data, data_file)
                    data_file.write('\n')  # Newline for each JSON object
                    data_file.flush()  # Ensure data is written to disk
                except queue.Empty:
                    continue  # No new data, continue looping
        except KeyboardInterrupt:
            logging.info("Detection stopped by user.")
        finally:
            # Stop threads
            mpu.stop()
            xm125.stop()
            hc_sr04.stop()

            # Wait for threads to finish
            mpu_thread.join()
            xm125_thread.join()
            hc_sr04_thread.join()

            # Clean up GPIO and close I2C bus
            GPIO.cleanup()
            bus.close()
            logging.info("Resources have been released.")

if __name__ == "__main__":
    main()