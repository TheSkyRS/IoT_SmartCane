import smbus2
import time

# MPU-6050 寄存器地址
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

class MPU6050:
    def __init__(self, bus=1, address=MPU6050_ADDR):
        """
        初始化 MPU-6050
        :param bus: I2C 总线号
        :param address: 设备 I2C 地址
        """
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def initialize(self, accel_range=0x00, gyro_range=0x00):
        """
        初始化 MPU-6050
        :param accel_range: 加速度量程配置 (默认 ±2 g)
        :param gyro_range: 陀螺仪量程配置 (默认 ±250 °/s)
        """
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)  # 唤醒设备
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, accel_range)  # 设置加速度量程
        self.bus.write_byte_data(self.address, GYRO_CONFIG, gyro_range)  # 设置陀螺仪量程

    def read_raw_data(self, reg):
        """
        从寄存器读取原始数据
        :param reg: 寄存器地址
        :return: 有符号整数值
        """
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def read_accelerometer(self):
        """
        读取加速度数据
        :return: (acc_x, acc_y, acc_z) (单位：g)
        """
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = self.read_raw_data(ACCEL_XOUT_H + 4)
        return acc_x * 0.000061, acc_y * 0.000061, acc_z * 0.000061  # 默认 ±2 g

    def read_gyroscope(self):
        """
        读取陀螺仪数据
        :return: (gyro_x, gyro_y, gyro_z) (单位：°/s)
        """
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
        return gyro_x * 0.00763, gyro_y * 0.00763, gyro_z * 0.00763  # 默认 ±250 °/s

    def close(self):
        """
        关闭 I2C 连接
        """
        self.bus.close()

# 校准函数
def calibrate_sensor(mpu, read_func, num_samples=100):
    """
    校准传感器
    :param mpu: MPU6050 对象
    :param read_func: 用于读取数据的函数
    :param num_samples: 采样次数
    :return: 偏移值 (offset_x, offset_y, offset_z)
    """
    offset_x, offset_y, offset_z = 0, 0, 0
    for _ in range(num_samples):
        x, y, z = read_func()
        offset_x += x
        offset_y += y
        offset_z += z
        time.sleep(0.01)  # 每次采样间隔
    return offset_x / num_samples, offset_y / num_samples, offset_z / num_samples

if __name__ == "__main__":
    mpu = MPU6050()
    mpu.initialize(accel_range=0x00, gyro_range=0x00)  # 默认配置：±2 g 和 ±250 °/s

    try:
        # 加速度校准
        print("正在校准加速度计...")
        acc_offset_x, acc_offset_y, acc_offset_z = calibrate_sensor(mpu, mpu.read_accelerometer)
        print(f"加速度计偏移: X={acc_offset_x:.4f}, Y={acc_offset_y:.4f}, Z={acc_offset_z:.4f}")

        # 陀螺仪校准
        print("正在校准陀螺仪...")
        gyro_offset_x, gyro_offset_y, gyro_offset_z = calibrate_sensor(mpu, mpu.read_gyroscope)
        print(f"陀螺仪偏移: X={gyro_offset_x:.4f}, Y={gyro_offset_y:.4f}, Z={gyro_offset_z:.4f}")

        while True:
            # 加速度校准前
            acc_x, acc_y, acc_z = mpu.read_accelerometer()
            print(f"加速度 (未校准): X={acc_x:.4f} g, Y={acc_y:.4f} g, Z={acc_z:.4f} g")

            # 加速度校准后
            acc_x -= acc_offset_x
            acc_y -= acc_offset_y
            acc_z -= acc_offset_z
            print(f"加速度 (校准后): X={acc_x:.4f} g, Y={acc_y:.4f} g, Z={acc_z:.4f} g")

            # 陀螺仪校准前
            gyro_x, gyro_y, gyro_z = mpu.read_gyroscope()
            print(f"陀螺仪 (未校准): X={gyro_x:.4f} °/s, Y={gyro_y:.4f} °/s, Z={gyro_z:.4f} °/s")

            # 陀螺仪校准后
            gyro_x -= gyro_offset_x
            gyro_y -= gyro_offset_y
            gyro_z -= gyro_offset_z
            print(f"陀螺仪 (校准后): X={gyro_x:.4f} °/s, Y={gyro_y:.4f} °/s, Z={gyro_z:.4f} °/s")

            time.sleep(1)

    except KeyboardInterrupt:
        print("退出")
    finally:
        mpu.close()
