import matplotlib.pyplot as plt
from pymongo import MongoClient
import time

# 连接到 MongoDB
client = MongoClient("mongodb://localhost:27017/")
db = client["sensor_data"]

# 可视化函数
def visualize_live_data():
    plt.ion()  # 开启交互模式
    fig, axes = plt.subplots(3, 1, figsize=(10, 15))

    while True:
        # 查询集合中的最近 200 条数据
        imu_data = list(db["imu_data"].find().sort("timestamp", -1).limit(100))
        ultrasonic_data = list(db["ultrasonic_data"].find().sort("timestamp", -1).limit(100))
        radar_data = list(db["radar_data"].find().sort("timestamp", -1).limit(100))

        # 清除当前图像
        for ax in axes:
            ax.clear()

        # 1. 更新 IMU 数据
        if imu_data:
            imu_timestamps = [item["timestamp"] for item in imu_data]
            imu_acc_changes = [item["acc_change"]["Total Change"] for item in imu_data]
            imu_gyro_changes = [item["gyro_change"]["Total Change"] for item in imu_data]

            axes[0].plot(imu_timestamps, imu_acc_changes, label="Acceleration Change")
            axes[0].plot(imu_timestamps, imu_gyro_changes, label="Gyro Change")
            axes[0].set_title("IMU Data (MPU6050)")
            axes[0].set_xlabel("Timestamp")
            axes[0].set_ylabel("Change")
            axes[0].legend()
            axes[0].grid()

        # 2. 更新超声波数据
        if ultrasonic_data:
            ultrasonic_timestamps = [item["timestamp"] for item in ultrasonic_data]
            ultrasonic_distances = [item["distance_m"] for item in ultrasonic_data]

            axes[1].plot(ultrasonic_timestamps, ultrasonic_distances, label="Distance (m)", color="orange")
            axes[1].set_title("Ultrasonic Data (HC_SR04)")
            axes[1].set_xlabel("Timestamp")
            axes[1].set_ylabel("Distance (m)")
            axes[1].legend()
            axes[1].grid()

        # 3. 更新毫米波数据
        if radar_data:
            filtered_data = []
            
            for item in radar_data:
                timestamp = item["timestamp"]
                for distance in item["nearest_distance"]:
                    if distance < 1.5:
                        filtered_data.append((timestamp, distance))
            
            filtered_timestamps = [item[0] for item in filtered_data]
            filtered_distances = [item[1] for item in filtered_data]

            # 更改绘图为点图并减小点的大小
            axes[2].scatter(filtered_timestamps, filtered_distances, s=10, label="Nearest Distance (m)", color="green")
            axes[2].set_title("Radar Data (XM125)")
            axes[2].set_xlabel("Timestamp")
            axes[2].set_ylabel("Nearest Distance (m)")
            axes[2].legend()
            axes[2].grid()





        # 刷新图像
        plt.pause(0.1)

# 调用可视化函数
try:
    visualize_live_data()
except KeyboardInterrupt:
    print("实时监控已停止。")

# 关闭 MongoDB 连接
client.close()
