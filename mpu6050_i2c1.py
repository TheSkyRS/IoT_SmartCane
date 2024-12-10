import smbus2
import time

# MPU-6050 Register Address
MPU6050_ADDR = 0x68  
PWR_MGMT_1 = 0x6B    
ACCEL_XOUT_H = 0x3B  
GYRO_XOUT_H = 0x43   
WHO_AM_I = 0x75      

class MPU6050:
    def __init__(self, bus=1, address=MPU6050_ADDR):
        """
        initialize MPU-6050
        :param bus: I2C 
        :param address: 
        """
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def initialize(self):
        """
        MPU-6050
        """
        # device
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        print("MPU-6050 ok")

        # ID
        device_id = self.bus.read_byte_data(self.address, WHO_AM_I)
        if device_id != 0x68:
            raise RuntimeError(f"ID not match: 0x{device_id:X}")
        print("MPU-6050 successful")

    def _read_raw_data(self, reg):
        
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value > 32767:  
            value -= 65536
        return value

    def read_accelerometer(self):
        """
        
        :return: (acc_x, acc_y, acc_z)
        """
        acc_x = self._read_raw_data(ACCEL_XOUT_H)
        acc_y = self._read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = self._read_raw_data(ACCEL_XOUT_H + 4)
        return acc_x, acc_y, acc_z

    def read_gyroscope(self):
        """
        
        :return: (gyro_x, gyro_y, gyro_z)
        """
        gyro_x = self._read_raw_data(GYRO_XOUT_H)
        gyro_y = self._read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self._read_raw_data(GYRO_XOUT_H + 4)
        return gyro_x, gyro_y, gyro_z

    def close(self):
        """
       
        """
        self.bus.close()
        print("I2C close")

# -------------------- test --------------------

if __name__ == "__main__":
    try:
        # inital MPU-6050
        mpu = MPU6050()
        mpu.initialize()

        while True:
            
            acc_x, acc_y, acc_z = mpu.read_accelerometer()
            gyro_x, gyro_y, gyro_z = mpu.read_gyroscope()

            print(f"acceleration: X={acc_x}, Y={acc_y}, Z={acc_z}")
            print(f"Gyroscope: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("measure stop")
    except Exception as e:
        print(f"error: {e}")
    finally:
        mpu.close()
