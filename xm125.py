from machine import I2C, Pin
import time
import struct

# ============================
# XM125(A121) PCR Radar Module
# MicroPython Script for ESP8266
# Optimized for Low Latency and High Accuracy
# ============================

# Constants
I2C_ADDR = 0x52  # Default I2C address; change if ADDR pin is connected differently

# Register Addresses
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

# Enumerations
class ThresholdMethod:
    FIXED_AMPLITUDE = 1
    FIXED_STRENGTH = 2
    CFAR = 3
    RECORDED = 4

class PeakSortingMethod:
    CLOSEST = 1
    STRONGEST = 2

class ReflectorShape:
    GENERIC = 1
    PLANAR = 2

class Profile:
    PROFILE1 = 1
    PROFILE2 = 2
    PROFILE3 = 3
    PROFILE4 = 4
    PROFILE5 = 5

# Detector Configuration Parameters
DetectorConfig = {
    'start_m': 0.1,  # meters (10 cm)
    'end_m': 3.0,     # meters (3 m)
    'max_step_length': 0,  # 0 means auto
    'max_profile': Profile.PROFILE1,
    'prf': None,  # PRF not defined in provided info
    'close_range_leakage_cancellation': False,  # Set to True if measuring close to sensor (<10 cm)
    'signal_quality': 15.0,  # dB
    'threshold_method': ThresholdMethod.CFAR,
    'peaksorting_method': PeakSortingMethod.STRONGEST,
    'reflector_shape': ReflectorShape.GENERIC,
    'num_frames_in_recorded_threshold': 100,
    'fixed_threshold_value': 100.0,  # factor 1000 larger
    'fixed_strength_threshold_value': 0.0,  # factor 1000 larger
    'threshold_sensitivity': 500,  # factor 1000 larger
    'update_rate': None,  # Not used in this script
    'measure_on_wakeup': False,  # Prevent KeyError
}

# GPIO Pins
WAKE_UP_PIN_NUM = 12  # GPIO12 (D6)
INT_PIN_NUM = 13      # GPIO13 (D7)

# Measurement Interval
MEASUREMENT_INTERVAL = 0.1  # seconds

# Initialize I2C (Adjust pins if necessary)
i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)  # GPIO5 (D1) as SCL, GPIO4 (D2) as SDA

# Initialize Control Pins
wake_up = Pin(WAKE_UP_PIN_NUM, Pin.OUT)
wake_up.value(0)  # Initially low
int_pin = Pin(INT_PIN_NUM, Pin.IN, Pin.PULL_UP)

def read_unsigned_register(reg_addr):
    """
    Reads a 32-bit unsigned register from the XM125 module.

    :param reg_addr: Register address (integer)
    :return: 32-bit unsigned integer value or None if failed
    """
    try:
        addr_bytes = struct.pack('>H', reg_addr)
        i2c.writeto(I2C_ADDR, addr_bytes)
        time.sleep_ms(10)  # Small delay before reading
        data = i2c.readfrom(I2C_ADDR, 4)
        if len(data) != 4:
            # print(f"Read error: Expected 4 bytes, got {len(data)}")  # Debug
            return None
        value = struct.unpack('>I', data)[0]
        return value
    except Exception as e:
        # print(f"Exception in read_unsigned_register (0x{reg_addr:X}): {e}")  # Debug
        return None

def read_signed_register(reg_addr):
    """
    Reads a 32-bit signed register from the XM125 module.

    :param reg_addr: Register address (integer)
    :return: 32-bit signed integer value or None if failed
    """
    try:
        addr_bytes = struct.pack('>H', reg_addr)
        i2c.writeto(I2C_ADDR, addr_bytes)
        time.sleep_ms(10)  # Small delay before reading
        data = i2c.readfrom(I2C_ADDR, 4)
        if len(data) != 4:
            # print(f"Read error: Expected 4 bytes, got {len(data)}")  # Debug
            return None
        value = struct.unpack('>i', data)[0]
        return value
    except Exception as e:
        # print(f"Exception in read_signed_register (0x{reg_addr:X}): {e}")  # Debug
        return None

def write_register(reg_addr, value):
    """
    Writes a 32-bit value to a register in the XM125 module.

    :param reg_addr: Register address (integer)
    :param value: 32-bit integer value to write
    :return: True if successful, False otherwise
    """
    try:
        data = struct.pack('>H', reg_addr) + struct.pack('>I', value)
        i2c.writeto(I2C_ADDR, data)
        return True
    except Exception as e:
        # print(f"Exception in write_register (0x{reg_addr:X}): {e}")  # Debug
        return False

def wait_for_int(timeout=10):
    """
    Waits for the INT pin to go HIGH indicating the module is ready.

    :param timeout: Timeout in seconds
    :return: True if INT is HIGH within timeout, False otherwise
    """
    start_time = time.time()
    # print("Waiting for INT pin to go HIGH...")  # Debug
    while time.time() - start_time < timeout:
        current_state = int_pin.value()
        # print(f"INT pin state: {current_state}")  # Debug
        if current_state == 1:
            return True
        time.sleep_ms(100)
    # print("Timeout waiting for INT pin.")  # Debug
    return False

def initialize_module():
    """
    Initializes the XM125 module by waking it up and ensuring it's ready.

    :return: True if successful, False otherwise
    """
    # Set WAKE UP high
    wake_up.value(1)
    # print("WAKE UP set to HIGH.")  # Debug

    # Wait for INT pin to go HIGH indicating ready state
    if not wait_for_int():
        # print("Module did not become ready.")  # Debug
        return False
    # print("Module is ready for I2C communication.")  # Debug
    return True

def write_default_configuration():
    """
    Writes the default configuration registers to the XM125 module.

    :return: True if successful, False otherwise
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

    # print("Writing default configuration...")  # Debug
    for reg, val in config_registers.items():
        success = write_register(reg, val)
        if not success:
            # print(f"Failed to write to register 0x{reg:04X}.")  # Debug
            return False
        # print(f"Register 0x{reg:04X} set to {val}.")  # Debug
    return True

def apply_config_and_calibrate():
    """
    Applies configuration settings and calibrates the module.

    :return: True if successful, False otherwise
    """
    if not write_default_configuration():
        # print("Failed to write default configuration.")  # Debug
        return False

    # Send APPLY_CONFIG_AND_CALIBRATE command
    if not write_register(0x0100, COMMAND_APPLY_CONFIG_AND_CALIBRATE):
        # print("Failed to send APPLY_CONFIG_AND_CALIBRATE command.")  # Debug
        return False
    # print("APPLY_CONFIG_AND_CALIBRATE command sent.")  # Debug

    # Wait for BUSY bit to clear
    # print("Waiting for module to finish configuration and calibration...")  # Debug
    while True:
        status = read_unsigned_register(0x0003)  # Detector Status
        if status is None:
            # print("Failed to read Detector Status.")  # Debug
            return False
        if not (status & 0x80000000):  # BUSY mask
            break
        time.sleep_ms(100)
    # print("Configuration and calibration completed.")  # Debug

    # Check for Detector Errors
    error_flags = (status >> 16) & 0xFF  # Bits 16 to 23
    if error_flags != 0:
        # print(f"Detector Status Error Flags: 0x{error_flags:02X}")  # Debug
        return False
    return True

def perform_recalibration():
    """
    Performs recalibration by sending the RECALIBRATE command.

    :return: True if successful, False otherwise
    """
    # Send RECALIBRATE command
    if not write_register(0x0100, COMMAND_RECALIBRATE):
        # print("Failed to send RECALIBRATE command.")  # Debug
        return False
    # print("RECALIBRATE command sent.")  # Debug

    # Wait for BUSY bit to clear
    # print("Waiting for module to finish recalibration...")  # Debug
    while True:
        status = read_unsigned_register(0x0003)  # Detector Status
        if status is None:
            # print("Failed to read Detector Status during recalibration.")  # Debug
            return False
        if not (status & 0x80000000):  # BUSY mask
            break
        time.sleep_ms(100)
    # print("Re-calibration completed.")  # Debug

    # Check for Detector Errors
    error_flags = (status >> 16) & 0xFF  # Bits 16 to 23
    if error_flags != 0:
        # print(f"Detector Status Error Flags during recalibration: 0x{error_flags:02X}")  # Debug
        return False
    return True

def measure_distance():
    """
    Triggers a distance measurement and reads the result.

    :return: Tuple (distances_m, strengths_db) or (None, None) if failed
    """
    # Send MEASURE_DISTANCE command
    if not write_register(0x0100, COMMAND_MEASURE_DISTANCE):
        # print("Failed to send MEASURE_DISTANCE command.")  # Debug
        return (None, None)
    # print("MEASURE_DISTANCE command sent.")  # Debug

    # Wait for BUSY bit to clear
    # print("Waiting for measurement to complete...")  # Debug
    while True:
        status = read_unsigned_register(0x0003)  # Detector Status
        if status is None:
            # print("Failed to read Detector Status.")  # Debug
            return (None, None)
        if not (status & 0x80000000):  # BUSY mask
            break
        time.sleep_ms(100)
    # print("Measurement completed.")  # Debug

    # Read Distance Result
    distance_result = read_unsigned_register(0x0010)
    if distance_result is None:
        # print("Failed to read Distance Result.")  # Debug
        return (None, None)

    # Parse Distance Result
    num_distances = distance_result & 0x0000000F
    near_start_edge = (distance_result >> 8) & 0x1
    calibration_needed = (distance_result >> 9) & 0x1
    measure_distance_error = (distance_result >> 10) & 0x1
    temperature = (distance_result >> 16) & 0xFFFF

    if measure_distance_error:
        # print("Measurement failed (MEASURE_DISTANCE_ERROR).")  # Debug
        return (None, None)

    if calibration_needed:
        # print("Calibration needed. Initiating re-calibration...")  # Debug
        if not perform_recalibration():
            # print("Re-calibration failed.")  # Debug
            return (None, None)
        # print("Re-calibration successful. Please retry the measurement.")  # Debug
        return (None, None)  # Skip this measurement

    if num_distances == 0:
        # print("No peaks detected.")  # Debug
        return ([], [])

    # Ensure num_distances does not exceed 10
    num_distances = min(num_distances, 10)
    # print(f"Number of distances detected: {num_distances}")  # Debug

    # Read Peaks
    distances_m = []
    strengths_db = []
    for i in range(num_distances):
        peak_dist_addr = 0x0011 + i  # 0x0011 to 0x001A
        peak_str_addr = 0x001B + i   # 0x001B to 0x0024

        distance_raw = read_unsigned_register(peak_dist_addr)
        strength_raw = read_signed_register(peak_str_addr)

        if distance_raw is not None and strength_raw is not None:
            # Convert to meters and dB respectively
            distance_m = distance_raw / 1000.0  # mm to meters
            strength_db_val = strength_raw / 1000.0  # dB


            distances_m.append(distance_m)
            strengths_db.append(strength_db_val)
            # print(f"Peak{i} - Distance: {distance_m:.3f} m | Strength: {strength_db_val:.3f} dB")  # Debug
        else:
            # print(f"Failed to read Peak{i} data.")  # Debug
            pass

    return (distances_m, strengths_db)

def main():
    """
    Main function to initialize the module, configure it, and continuously perform measurements.
    """
    # print("Initializing I2C...")  # Debug
    try:
        devices = i2c.scan()
        if I2C_ADDR not in devices:
            # print(f"Device with address 0x{I2C_ADDR:X} not found on the I2C bus.")  # Debug
            return
        # print(f"Found device at address 0x{I2C_ADDR:X}.")  # Debug
    except Exception as e:
        # print(f"I2C scan failed: {e}")  # Debug
        return

    # print("Initializing module...")  # Debug
    if not initialize_module():
        # print("Module initialization failed.")  # Debug
        return

    # print("Applying configuration and calibrating...")  # Debug
    if not apply_config_and_calibrate():
        # print("Configuration and calibration failed.")  # Debug
        return

    # print("Entering measurement loop. Press Ctrl+C to exit.")  # Debug
    try:
        while True:
            distances, strengths = measure_distance()
            if distances is not None and strengths is not None:
                if len(distances) > 0:
                    # print(f"Measured {len(distances)} peak(s).")  # Debug
                    for idx, (dist, stren) in enumerate(zip(distances, strengths)):
                        print(f"Peak{idx}: Distance = {dist:.3f} m | Strength = {stren:.3f} dB")
                else:
                    # print("No peaks detected in this measurement.")  # Debug
                    pass
            else:
                # print("Measurement skipped or failed.")  # Debug
                pass
            time.sleep(MEASUREMENT_INTERVAL)
    except KeyboardInterrupt:
        print("Measurement loop terminated by user.")

if __name__ == "__main__":
    main()