import smbus2
import time
import math

# MPU-6050 
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050:
    def __init__(self, bus=1, address=MPU6050_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def initialize(self):
        
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)

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

def calculate_change(prev, current):
   
    dx = current[0] - prev[0]
    dy = current[1] - prev[1]
    dz = current[2] - prev[2]
    total_change = math.sqrt(dx**2 + dy**2 + dz**2) 
    return dx, dy, dz, total_change

if __name__ == "__main__":
    mpu = MPU6050()
    mpu.initialize()

    try:
   
        prev_acc = mpu.read_accelerometer()
        prev_gyro = mpu.read_gyroscope()

        while True:
           
            current_acc = mpu.read_accelerometer()
            current_gyro = mpu.read_gyroscope()

           
            acc_dx, acc_dy, acc_dz, acc_total = calculate_change(prev_acc, current_acc)
            gyro_dx, gyro_dy, gyro_dz, gyro_total = calculate_change(prev_gyro, current_gyro)

            
            print(f"accelerometer: ΔX={acc_dx:.4f} g, ΔY={acc_dy:.4f} g, ΔZ={acc_dz:.4f} g, total_change={acc_total:.4f} g")
            print(f"gyroscope: ΔX={gyro_dx:.4f} °/s, ΔY={gyro_dy:.4f} °/s, ΔZ={gyro_dz:.4f} °/s, total_change={gyro_total:.4f} °/s")

         
            prev_acc = current_acc
            prev_gyro = current_gyro

            time.sleep(0.1)  

    except KeyboardInterrupt:
        print("detection stop")
    finally:
        mpu.close()
