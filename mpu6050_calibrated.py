import smbus2
import time

# MPU-6050 
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

class MPU6050:
    def __init__(self, bus=1, address=MPU6050_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def initialize(self, accel_range=0x00, gyro_range=0x00):
      
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00) 
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, accel_range) 
        self.bus.write_byte_data(self.address, GYRO_CONFIG, gyro_range)  

    def read_raw_data(self, reg):
       
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
        return acc_x * 0.000061, acc_y * 0.000061, acc_z * 0.000061  

    def read_gyroscope(self):
       
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
        return gyro_x * 0.00763, gyro_y * 0.00763, gyro_z * 0.00763 

    def close(self):
 
        self.bus.close()


def calibrate_sensor(mpu, read_func, num_samples=100):

    offset_x, offset_y, offset_z = 0, 0, 0
    for _ in range(num_samples):
        x, y, z = read_func()
        offset_x += x
        offset_y += y
        offset_z += z
        time.sleep(0.01) 
    return offset_x / num_samples, offset_y / num_samples, offset_z / num_samples

if __name__ == "__main__":
    mpu = MPU6050()
    mpu.initialize(accel_range=0x00, gyro_range=0x00)  

    try:
        # accelerometer
        print("Calibrating accelerometer...")
        acc_offset_x, acc_offset_y, acc_offset_z = calibrate_sensor(mpu, mpu.read_accelerometer)
        print(f"Accelerometer Offset: X={acc_offset_x:.4f}, Y={acc_offset_y:.4f}, Z={acc_offset_z:.4f}")

        # gyroscope
        print("Calibrating gyroscope...")
        gyro_offset_x, gyro_offset_y, gyro_offset_z = calibrate_sensor(mpu, mpu.read_gyroscope)
        print(f"Gyroscope Offset: X={gyro_offset_x:.4f}, Y={gyro_offset_y:.4f}, Z={gyro_offset_z:.4f}")

        while True:
            
            acc_x, acc_y, acc_z = mpu.read_accelerometer()
            print(f"accelerometer (Uncalibrated): X={acc_x:.4f} g, Y={acc_y:.4f} g, Z={acc_z:.4f} g")

            
            acc_x -= acc_offset_x
            acc_y -= acc_offset_y
            acc_z -= acc_offset_z
            print(f"accelerometer (calibrated): X={acc_x:.4f} g, Y={acc_y:.4f} g, Z={acc_z:.4f} g")

           
            gyro_x, gyro_y, gyro_z = mpu.read_gyroscope()
            print(f"gyroscope (Uncalibrated): X={gyro_x:.4f} °/s, Y={gyro_y:.4f} °/s, Z={gyro_z:.4f} °/s")

          
            gyro_x -= gyro_offset_x
            gyro_y -= gyro_offset_y
            gyro_z -= gyro_offset_z
            print(f"gyroscope (calibrated): X={gyro_x:.4f} °/s, Y={gyro_y:.4f} °/s, Z={gyro_z:.4f} °/s")

            time.sleep(1)

    except KeyboardInterrupt:
        print("exit")
    finally:
        mpu.close()
