from pymongo import MongoClient
import json
import time

# 连接到 MongoDB
client = MongoClient("mongodb://localhost:27017/")  # 替换为你的 MongoDB URI
db = client["sensor_data"]  # 数据库名称
imu_collection = db["imu_data"]  # IMU 数据集合
ultrasonic_collection = db["ultrasonic_data"]  # 超声波数据集合
radar_collection = db["radar_data"]  # 毫米波数据集合

# 在程序执行前清空集合
imu_collection.delete_many({})
ultrasonic_collection.delete_many({})
radar_collection.delete_many({})
print("已清空以前存储的内容。")


time.sleep(10)

# 文件路径
file_path = 'sensor_data.jsonl'

# 初始化计数器
line_counter = 0

# 读取文件并处理数据
with open(file_path, "r", encoding="utf-8") as file:
    for line in file:
        line_counter += 1  # 增加行号计数器
        line = line.strip()

        if not line:  # 跳过空行
            print(f"第 {line_counter} 行是空行，跳过。")
            continue
        
        try:
            # 解析 JSON 数据
            data = json.loads(line)

            # 打印当前行和数据种类
            print(f"第 {line_counter} 行: 数据类型 -> {data.get('type', '未知')}")

            # 根据数据类型存储到不同的集合
            if data["type"] == "MPU6050":
                imu_collection.insert_one({
                    "acc_change": data["acc_change"],
                    "gyro_change": data["gyro_change"],
                    "timestamp": data["timestamp"]
                })
            elif data["type"] == "HC_SR04":
                ultrasonic_collection.insert_one({
                    "distance_m": data["distance_m"],
                    "timestamp": data["timestamp"]
                })
            elif data["type"] == "XM125":
                # 检查 distances_m 是否为空
                if not data["distances_m"]:
                    print(f"第 {line_counter} 行: XM125 数据的 distances_m 列表为空，跳过。")
                    continue
                radar_collection.insert_one({
                    "nearest_distance": data["distances_m"],
                    "strengths_db": data["strengths_db"],
                    "timestamp": data["timestamp"]
                })
        except json.JSONDecodeError as e:
            print(f"第 {line_counter} 行解析错误: {e}")
        except KeyError as e:
            print(f"第 {line_counter} 行缺少关键字段 {e}，跳过。")

print("数据已成功存入 MongoDB！")

# 关闭 MongoDB 连接（可选）
client.close()
