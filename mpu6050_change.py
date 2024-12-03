import smbus2
import time
import math

# MPU-6050 寄存器地址
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050:
    def __init__(self, bus=1, address=MPU6050_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def initialize(self):
        # 唤醒设备
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
        # 默认 ±2 g 模式，将原始值转换为 g
        return acc_x * 0.000061, acc_y * 0.000061, acc_z * 0.000061

    def read_gyroscope(self):
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
        # 默认 ±250 °/s 模式，将原始值转换为 °/s
        return gyro_x * 0.00763, gyro_y * 0.00763, gyro_z * 0.00763

    def close(self):
        self.bus.close()

def calculate_change(prev, current):
    """
    计算三轴变化的幅度
    :param prev: 前一次读数 (x, y, z)
    :param current: 当前读数 (x, y, z)
    :return: 各轴变化幅度, 总变化幅度
    """
    dx = current[0] - prev[0]
    dy = current[1] - prev[1]
    dz = current[2] - prev[2]
    total_change = math.sqrt(dx**2 + dy**2 + dz**2)  # 欧几里得距离
    return dx, dy, dz, total_change

if __name__ == "__main__":
    mpu = MPU6050()
    mpu.initialize()

    try:
        # 初始化前一次加速度和陀螺仪读数
        prev_acc = mpu.read_accelerometer()
        prev_gyro = mpu.read_gyroscope()

        while True:
            # 当前加速度和陀螺仪读数
            current_acc = mpu.read_accelerometer()
            current_gyro = mpu.read_gyroscope()

            # 计算变化幅度
            acc_dx, acc_dy, acc_dz, acc_total = calculate_change(prev_acc, current_acc)
            gyro_dx, gyro_dy, gyro_dz, gyro_total = calculate_change(prev_gyro, current_gyro)

            # 输出加速度变化
            print(f"加速度变化: ΔX={acc_dx:.4f} g, ΔY={acc_dy:.4f} g, ΔZ={acc_dz:.4f} g, 总变化={acc_total:.4f} g")

            # 输出陀螺仪变化
            print(f"陀螺仪变化: ΔX={gyro_dx:.4f} °/s, ΔY={gyro_dy:.4f} °/s, ΔZ={gyro_dz:.4f} °/s, 总变化={gyro_total:.4f} °/s")

            # 更新前一次读数
            prev_acc = current_acc
            prev_gyro = current_gyro

            time.sleep(0.1)  # 采样间隔 100 毫秒

    except KeyboardInterrupt:
        print("检测已停止")
    finally:
        mpu.close()
