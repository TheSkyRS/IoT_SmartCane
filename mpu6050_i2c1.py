import smbus2
import time

# MPU-6050 寄存器地址
MPU6050_ADDR = 0x68  # 默认 I2C 地址 (AD0 接地)
PWR_MGMT_1 = 0x6B    # 电源管理寄存器
ACCEL_XOUT_H = 0x3B  # 加速度 X 高字节寄存器
GYRO_XOUT_H = 0x43   # 陀螺仪 X 高字节寄存器
WHO_AM_I = 0x75      # 设备 ID 寄存器

class MPU6050:
    def __init__(self, bus=1, address=MPU6050_ADDR):
        """
        初始化 MPU-6050
        :param bus: I2C 总线号
        :param address: 设备 I2C 地址
        """
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def initialize(self):
        """
        初始化 MPU-6050
        """
        # 唤醒设备
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        print("MPU-6050 已唤醒")

        # 验证设备 ID
        device_id = self.bus.read_byte_data(self.address, WHO_AM_I)
        if device_id != 0x68:
            raise RuntimeError(f"设备 ID 不匹配: 0x{device_id:X}")
        print("MPU-6050 初始化成功")

    def _read_raw_data(self, reg):
        """
        从寄存器读取原始数据
        :param reg: 寄存器地址
        :return: 有符号整数值
        """
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value > 32767:  # 处理负值
            value -= 65536
        return value

    def read_accelerometer(self):
        """
        读取加速度数据
        :return: (acc_x, acc_y, acc_z)
        """
        acc_x = self._read_raw_data(ACCEL_XOUT_H)
        acc_y = self._read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = self._read_raw_data(ACCEL_XOUT_H + 4)
        return acc_x, acc_y, acc_z

    def read_gyroscope(self):
        """
        读取陀螺仪数据
        :return: (gyro_x, gyro_y, gyro_z)
        """
        gyro_x = self._read_raw_data(GYRO_XOUT_H)
        gyro_y = self._read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self._read_raw_data(GYRO_XOUT_H + 4)
        return gyro_x, gyro_y, gyro_z

    def close(self):
        """
        关闭 I2C 连接
        """
        self.bus.close()
        print("I2C 连接已关闭")

# -------------------- 测试代码 --------------------

if __name__ == "__main__":
    try:
        # 初始化 MPU-6050
        mpu = MPU6050()
        mpu.initialize()

        while True:
            # 读取加速度和陀螺仪数据
            acc_x, acc_y, acc_z = mpu.read_accelerometer()
            gyro_x, gyro_y, gyro_z = mpu.read_gyroscope()

            print(f"加速度: X={acc_x}, Y={acc_y}, Z={acc_z}")
            print(f"陀螺仪: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("测量已停止")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        mpu.close()
