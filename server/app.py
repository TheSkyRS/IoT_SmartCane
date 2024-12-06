from flask import Flask, request, jsonify, render_template, send_from_directory
from pymongo import MongoClient
from datetime import datetime
import threading
import requests
import queue
import time
from werkzeug.utils import secure_filename
import os
# 导入 ZoneInfo 用于时区处理（Python 3.9+）
try:
    from zoneinfo import ZoneInfo
except ImportError:
    # 如果使用的是 Python 版本低于 3.9，使用 pytz
    import pytz
    from pytz import timezone

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
# 音频文件上传配置
# ============================

# 定义允许上传的音频文件扩展名
ALLOWED_AUDIO_EXTENSIONS = {'wav', 'mp3', 'ogg'}

# 定义音频文件的上传文件夹
AUDIO_UPLOAD_FOLDER = 'audio'
os.makedirs(AUDIO_UPLOAD_FOLDER, exist_ok=True)  # 确保音频文件夹存在

# 配置 Flask 应用
app.config['AUDIO_UPLOAD_FOLDER'] = AUDIO_UPLOAD_FOLDER
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024  # 最大上传文件大小为16MB

def allowed_audio_file(filename):
    """
    检查文件是否为允许的音频格式。
    """
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_AUDIO_EXTENSIONS

def save_audio_file(file):
    """
    保存上传的音频文件到指定文件夹，并生成基于系统时间的唯一文件名（纽约时间，格式：YYYY-MM-DD_HH-MM-SS）。
    """
    if file and allowed_audio_file(file.filename):
        filename = secure_filename(file.filename)
        
        # 获取当前纽约时间
        try:
            now_ny = datetime.now(ZoneInfo('America/New_York'))
        except NameError:
            # 如果 zoneinfo 不可用，使用 pytz
            ny_tz = timezone('America/New_York')
            now_ny = datetime.now(ny_tz)
        
        # 格式化时间戳
        timestamp = now_ny.strftime("%Y-%m-%d_%H-%M-%S")
        unique_filename = f"{timestamp}_{filename}"
        file_path = os.path.join(app.config['AUDIO_UPLOAD_FOLDER'], unique_filename)
        file.save(file_path)
        print(f"[INFO] Audio File Saved: {file_path}")
        return unique_filename
    else:
        print("[ERROR] Invalid Audio File Type")
        return None

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

@app.route('/audio', methods=['POST'])
def upload_audio():
    """
    接收音频文件的端点，仅处理文件上传，不处理传感器数据。
    """
    if 'file' not in request.files:
        return jsonify({"error": "No file part in the request."}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "No selected file."}), 400

    saved_filename = save_audio_file(file)
    if saved_filename:
        return jsonify({"status": "success", "message": f"File '{saved_filename}' uploaded successfully."}), 200
    else:
        return jsonify({"error": "Invalid audio file type."}), 400

@app.route('/fall_audios')
def fall_audios():
    """
    渲染包含所有摔倒音频文件的页面。
    """
    try:
        # 获取所有音频文件
        audio_files = os.listdir(app.config['AUDIO_UPLOAD_FOLDER'])
        # 过滤出包含 'fall_audio' 的文件（如果有特定命名规则）
        fall_audio_files = [f for f in audio_files if 'fall_audio' in f]
        # 按时间排序（假设文件名中包含时间戳）
        fall_audio_files.sort(reverse=True)  # 最新的在前面
    except Exception as e:
        print(f"[ERROR] Failed to fetch audio files: {e}")
        fall_audio_files = []
    
    return render_template('fall_audios.html', audio_files=fall_audio_files)

@app.route('/audio/<filename>')
def serve_audio(filename):
    """
    提供音频文件的端点，用于播放音频。
    """
    return send_from_directory(app.config['AUDIO_UPLOAD_FOLDER'], filename)

if __name__ == '__main__':
    # gunicorn -w 4 -b 0.0.0.0:5000 app:app
    app.run(host='0.0.0.0', port=5000)
