from flask import Flask, request, jsonify, render_template
from pymongo import MongoClient
from datetime import datetime
import threading
import requests
import queue
import time

app = Flask(__name__)


client = MongoClient('mongodb://localhost:27017/') 
db = client['sensor_data']

hc_sr04_collection = db['hc_sr04']
mpu6050_collection = db['mpu6050']
xm125_collection = db['xm125']

def enforce_collection_limit_async(collection, limit=1000, delete_count=900):
    thread = threading.Thread(target=enforce_collection_limit, args=(collection, limit, delete_count))
    thread.start()

def enforce_collection_limit(collection, limit=1000, delete_count=900):
    count = collection.count_documents({})
    if count > limit:
        oldest_documents = collection.find().sort("timestamp", -1).limit(delete_count)
        oldest_ids = [doc['_id'] for doc in oldest_documents]
        result = collection.delete_many({"_id": {"$in": oldest_ids}})

# ============================
# API 端点
# ============================

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data')
def get_data():
    # 查询每个集合中的最近 100 条数据
    imu_data = list(mpu6050_collection.find().sort("timestamp", -1).limit(100))
    ultrasonic_data = list(hc_sr04_collection.find().sort("timestamp", -1).limit(100))
    radar_data = list(xm125_collection.find().sort("timestamp", -1).limit(100))
    
    # 将数据按时间顺序排列（从旧到新）
    imu_data = imu_data[::-1]
    ultrasonic_data = ultrasonic_data[::-1]
    radar_data = radar_data[::-1]
    
    # 处理数据以适应前端需求
    response = {
        "mpu6050": [
            {
                "timestamp": item["timestamp"],
                "acc_change": item["acc_change"]["Total Change"],
                "gyro_change": item["gyro_change"]["Total Change"]
            }
            for item in imu_data
        ],
        "hcsr04": [
            {
                "timestamp": item["timestamp"],
                "distance_m": item["distance_m"]
            }
            for item in ultrasonic_data
        ],
        "xm125": [
            {
                "timestamp": item["timestamp"],
                "distances_m": item["distances_m"]
            }
            for item in radar_data
        ]
    }
    
    return jsonify(response)

@app.route('/data', methods=['POST'])
def receive_data():
    data_pack = request.get_json()
    if not isinstance(data_pack, list):
        return jsonify({"error": "DataPack must be a list"}), 400

    hc_sr04_data = []
    mpu6050_data = []
    xm125_data = []

    for data in data_pack:
        sensor_type = data.get('type')
        if sensor_type == 'HC_SR04':
            hc_sr04_data.append(data)
        elif sensor_type == 'MPU6050':
            mpu6050_data.append(data)
        elif sensor_type == 'XM125':
            xm125_data.append(data)
        else:
            return jsonify({"error": "Unknown sensor type"}), 400

    if hc_sr04_data:
        hc_sr04_collection.insert_many(hc_sr04_data)
        enforce_collection_limit_async(hc_sr04_collection)
    if mpu6050_data:
        mpu6050_collection.insert_many(mpu6050_data)
        enforce_collection_limit_async(mpu6050_collection)
    if xm125_data:
        xm125_collection.insert_many(xm125_data)
        enforce_collection_limit_async(xm125_collection)

        return jsonify({"status": "success"}), 200

if __name__ == '__main__':
    # gunicorn -w 4 -b 0.0.0.0:5000 app:app
    app.run(host='0.0.0.0', port=5000)
