#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
综合处理 HC_SR04、MPU6050 和 XM125 传感器数据的实时算法，
包括事件检测、音频提示和障碍物距离提示。
"""

import json
import threading
import sounddevice as sd
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import time
import warnings
from scipy.io import wavfile
from scipy.io.wavfile import WavFileWarning
import queue
import numpy as np
from collections import deque
import statistics
import requests
import queue


# 忽略 WavFileWarning 警告
warnings.filterwarnings("ignore", category=WavFileWarning)

# ============================
# 配置参数
# ============================

DATA_FILE = 'sensor_data.jsonl'

# HC_SR04 阈值设置
UP_STEP_DISTANCE_THRESHOLD = 0.2     # 米，小于此距离认为是上台阶
DOWN_STEP_DISTANCE_THRESHOLD = 0.5   # 米，大于此距离认为是下台阶
HC_SR04_DEBOUNCE_TIME = 1.0          # 秒，防抖时间
HC_SR04_COOLDOWN_TIME = 3.0          # 秒，提示音冷却时间
HC_SR04_BUFFER_SIZE = 5
HC_SR04_MAX_THRESHOLD = 2 # 米

# MPU6050 阈值设置
FALL_ACCEL_THRESHOLD = 2.5           # g，加速度阈值，超过此值认为是摔倒
FALL_GYRO_THRESHOLD = 300            # °/s，角速度阈值
FALL_DEBOUNCE_TIME = 2.0             # 秒，防抖时间
FALL_COOLDOWN_TIME = 5.0             # 秒，提示音冷却时间

STILL_ACCEL_THRESHOLD = 0.5  # g，示例值
STILL_GYRO_THRESHOLD = 50.0  # °/s，示例值

# XM125 阈值设置
OBSTACLE_DISTANCE_THRESHOLD = 1.2    # 米
OBSTACLE_DISTANCE_MIN = 0.1          # 米

# 频率映射
FREQ_MAX = 4000  # Hz
FREQ_MIN = 200   # Hz

# 声音参数
SAMPLE_RATE = 44100  # 采样率

# 音频文件路径
UP_AUDIO_FILE = 'up.wav'
DOWN_AUDIO_FILE = 'down.wav'
FALL_AUDIO_FILE = 'fall.wav'

# 警告音优先级
ALERT_PRIORITIES = {
    FALL_AUDIO_FILE: 1,   # 最高优先级
    UP_AUDIO_FILE: 2,
    DOWN_AUDIO_FILE: 2
}

CLOUD_SERVER_URL = 'http://34.72.243.54:5000/data'
BATCH_SIZE = 10
BATCH_TIMEOUT = 3
QUEUE_SIZE = 1000

# ============================
# 辅助函数和类
# ============================

def map_distance_to_frequency(distance):
    """
    将障碍物距离映射到频率范围内。
    """
    freq = FREQ_MIN + (OBSTACLE_DISTANCE_THRESHOLD - distance) * \
        (FREQ_MAX - FREQ_MIN) / (OBSTACLE_DISTANCE_THRESHOLD - OBSTACLE_DISTANCE_MIN)
    freq = np.clip(freq, FREQ_MIN, FREQ_MAX)
    return freq

class AudioPlayer:
    """
    管理音频播放，包括左声道的频率声音和右声道的预加载警告音。
    """
    def __init__(self, sample_rate=44100, device=None):
        self.sample_rate = sample_rate
        self.frequency = 0               # 左声道频率
        self.phase = 0
        self.lock = threading.Lock()
        self.device = device
        self.stream = sd.OutputStream(
            channels=2,  # 立体声
            callback=self.audio_callback,
            samplerate=self.sample_rate,
            device=self.device,
            blocksize=0,
            dtype='float32'
        )
        self.alert_lock = threading.Lock()
        self.preloaded_sounds = {}      # 预加载的音频数据
        self.alert_queue = queue.PriorityQueue()
        self.current_alert_sound = None
        self.alert_sound_position = 0
        self.alerts_in_queue = set()    # 记录已在队列中的警告音

        # 预加载警告音频
        for sound_file in [UP_AUDIO_FILE, DOWN_AUDIO_FILE, FALL_AUDIO_FILE]:
            if not os.path.exists(sound_file):
                print(f"[ERROR] Audio file '{sound_file}' not found.")
                continue
            samplerate, data = wavfile.read(sound_file)
            data = data.astype(np.float32) / np.iinfo(data.dtype).max  # 归一化数据
            if data.ndim > 1 and data.shape[1] > 1:
                data = data.mean(axis=1)  # 将立体声转换为单声道
            self.preloaded_sounds[sound_file] = data
            print(f"[DEBUG] Loaded '{sound_file}' successfully.")

    def start(self):
        self.stream.start()
        print("[INFO] Audio stream started.")

    def stop(self):
        self.stream.stop()
        self.stream.close()
        print("[INFO] Audio stream stopped.")

    def play_alert_sound(self, sound_file):
        """
        添加警告音到队列中，避免重复添加。
        """
        priority = ALERT_PRIORITIES.get(sound_file, 10)  # 默认优先级为 10
        with self.alert_lock:
            if sound_file not in self.alerts_in_queue:
                self.alert_queue.put((priority, sound_file))
                self.alerts_in_queue.add(sound_file)
                # print(f"[DEBUG] Added '{sound_file}' to alert queue with priority {priority}.")

    def audio_callback(self, outdata, frames, time_info, status):
        """
        音频回调函数，根据当前状态播放警告音或频率声音。
        """
        if status:
            print(f"[WARNING] {status}")

        # 处理右声道的警告音
        with self.alert_lock:
            if self.current_alert_sound is None and not self.alert_queue.empty():
                # 获取优先级最高的警告音
                _, sound_file = self.alert_queue.get()
                self.current_alert_sound = self.preloaded_sounds.get(sound_file)
                self.alert_sound_position = 0
                self.alerts_in_queue.discard(sound_file)
                # print(f"[DEBUG] Now playing '{sound_file}' on right channel.")

        # 左声道：频率声音
        with self.lock:
            freq = self.frequency

        if freq > 0:
            t = (np.arange(frames) + self.phase) / self.sample_rate
            left_channel = np.sin(2 * np.pi * freq * t).astype(np.float32)
            # print(f"[DEBUG] Playing frequency sound at {freq:.2f} Hz on left channel.")
        else:
            left_channel = np.zeros(frames, dtype=np.float32)

        self.phase = (self.phase + frames) % self.sample_rate

        # 右声道：警告音或静音
        if self.current_alert_sound is not None:
            data = self.current_alert_sound
            start = self.alert_sound_position
            end = start + frames
            chunk = data[start:end]
            if len(chunk) < frames:
                # 警告音播放完毕，填充剩余部分
                right_channel = np.concatenate((chunk, np.zeros(frames - len(chunk), dtype=np.float32)))
                # print(f"[DEBUG] Finished playing alert sound on right channel.")
                self.current_alert_sound = None
            else:
                right_channel = chunk
                self.alert_sound_position += frames
        else:
            # 没有警告音，静音
            right_channel = np.zeros(frames, dtype=np.float32)

        # 组合左右声道
        outdata[:, 0] = left_channel
        outdata[:, 1] = right_channel

class SensorDataProcessor:
    """
    处理传感器数据，检测事件并触发音频提示。
    """
    def __init__(self, audio_player):
        self.audio_player = audio_player
        self.last_hc_sr04_event_time = 0
        self.hc_sr04_distance_buffer = deque(maxlen=HC_SR04_BUFFER_SIZE)
        self.last_fall_event_time = 0
        self.last_hc_sr04_state = 'unknown'  # 初始状态为未知
        self.last_alert_times = {
            UP_AUDIO_FILE: 0,
            DOWN_AUDIO_FILE: 0,
            FALL_AUDIO_FILE: 0
        }
        self.movement_state = 'unknown'  # 初始运动状态
        
        # 发送队列和线程初始化
        self.send_queue = queue.Queue(maxsize=QUEUE_SIZE)
        self.batch_size = BATCH_SIZE  # 每批发送的数据量
        self.batch_timeout = BATCH_TIMEOUT  # 最大等待时间（秒）发送数据，即使未达到 batch_size
        self.sender_thread = threading.Thread(target=self._send_worker, daemon=True)
        self.sender_thread.start()
    
    def _send_worker(self):
        """
        发送工作线程，从队列中取出数据并批量发送到云端服务器。
        """
        data_pack = []
        last_send_time = time.time()
        while True:
            try:
                # 等待数据，超时后继续检查是否有需要发送的数据
                data = self.send_queue.get(timeout=self.batch_timeout)
                if data is None:
                    # 接收到退出信号
                    if data_pack:
                        self._send_data_pack(data_pack)
                    break
                data_pack.append(data)
                if len(data_pack) >= self.batch_size:
                    self._send_data_pack(data_pack)
                    data_pack = []
                    last_send_time = time.time()
            except queue.Empty:
                # 超时检查是否有数据需要发送
                current_time = time.time()
                if data_pack and (current_time - last_send_time) >= self.batch_timeout:
                    self._send_data_pack(data_pack)
                    data_pack = []
                    last_send_time = current_time

    def _send_data_pack(self, data_pack):
        """
        发送数据包到云端服务器。
        """
        try:
            response = requests.post(CLOUD_SERVER_URL, json=data_pack, timeout=BATCH_TIMEOUT)
            response.raise_for_status()
            print(f"[INFO] Send {len(data_pack)} datas to GCP.")
        except requests.exceptions.RequestException as e:
            print(f"[ERROR] Failed to send data_pack to GCP: {e}")

    def send_data_to_cloud(self, data):
        """
        将传感器数据放入发送队列中。
        """
        try:
            self.send_queue.put_nowait(data)
        except queue.Full:
            print("[WARNING] Queue is Full.")

    def process_data_entry(self, data):
        sensor_type = data.get('type')

        if sensor_type == 'HC_SR04':
            self.process_hc_sr04_data(data)
        elif sensor_type == 'MPU6050':
            self.process_mpu6050_data(data)
        elif sensor_type == 'XM125':
            self.process_xm125_data(data)
        else:
            print(f"[DEBUG] Unknown sensor type: {sensor_type}")

        # 将数据发送到云端
        self.send_data_to_cloud(data)

    def shutdown(self):
        """
        关闭发送线程，确保所有数据都被发送。
        """
        self.send_queue.put(None)  # 发送退出信号
        self.sender_thread.join()

    def process_hc_sr04_data(self, data):
        current_time = time.time()
        distance = data.get('distance_m')
        if distance is None:
            print("[DEBUG] HC_SR04 data missing 'distance_m'.")
            return

        # 运动状态不处理超声波
        if self.movement_state != 'still':
            print("[DEBUG] Skipping HC_SR04 processing because device is moving.")
            return
        
        self.hc_sr04_distance_buffer.append(distance)
        if len(self.hc_sr04_distance_buffer) < self.hc_sr04_distance_buffer.maxlen:
            return
        
        median_distance = statistics.median(self.hc_sr04_distance_buffer)
        data['distance_m'] = median_distance
        # 打印距离信息
        print(f"[DEBUG] HC_SR04 median distance: {median_distance:.2f} meters")
        # self.hc_sr04_distance_buffer.clear()

        # 防抖处理
        if current_time - self.last_hc_sr04_event_time < HC_SR04_DEBOUNCE_TIME:
            # print("[DEBUG] HC_SR04 event ignored due to debounce.")
            return

        # 确定当前状态
        if median_distance < UP_STEP_DISTANCE_THRESHOLD:
            current_state = 'up'
        elif median_distance > DOWN_STEP_DISTANCE_THRESHOLD and median_distance < HC_SR04_MAX_THRESHOLD:
            current_state = 'down'
        else:
            current_state = 'normal'

        # 状态变化检测
        if current_state != self.last_hc_sr04_state:
            if current_state == 'up':
                # 冷却时间检查
                if current_time - self.last_alert_times[UP_AUDIO_FILE] >= HC_SR04_COOLDOWN_TIME:
                    # 播放上台阶提示音
                    self.audio_player.play_alert_sound(UP_AUDIO_FILE)
                    self.last_alert_times[UP_AUDIO_FILE] = current_time
                    print("[INFO] Detected upward step. Playing 'up' audio on right channel.")
            elif current_state == 'down':
                # 冷却时间检查
                if current_time - self.last_alert_times[DOWN_AUDIO_FILE] >= HC_SR04_COOLDOWN_TIME:
                    # 播放下楼梯提示音
                    self.audio_player.play_alert_sound(DOWN_AUDIO_FILE)
                    self.last_alert_times[DOWN_AUDIO_FILE] = current_time
                    print("[INFO] Detected downward step. Playing 'down' audio on right channel.")

        # 更新上一次状态
        self.last_hc_sr04_state = current_state
        self.last_hc_sr04_event_time = current_time

    def process_mpu6050_data(self, data):
        current_time = time.time()
        acc_change = data.get('acc_change', {})
        gyro_change = data.get('gyro_change', {})
        acc_total_change = acc_change.get('Total Change', 0)
        gyro_total_change = gyro_change.get('Total Change', 0)

        # 打印加速度和角速度变化信息
        # print(f"[DEBUG] MPU6050 Acc Total Change: {acc_total_change:.2f} g")
        # print(f"[DEBUG] MPU6050 Gyro Total Change: {gyro_total_change:.2f} °/s")

        if acc_total_change >= FALL_ACCEL_THRESHOLD or gyro_total_change >= FALL_GYRO_THRESHOLD:
            # 防抖处理
            if current_time - self.last_fall_event_time < FALL_DEBOUNCE_TIME:
                # print("[DEBUG] MPU6050 fall event ignored due to debounce.")
                return
            # 冷却时间检查
            if current_time - self.last_alert_times[FALL_AUDIO_FILE] >= FALL_COOLDOWN_TIME:
                # 检测到摔倒，播放提示音
                self.audio_player.play_alert_sound(FALL_AUDIO_FILE)
                self.last_alert_times[FALL_AUDIO_FILE] = current_time
                print("[INFO] Fall detected. Playing 'fall' audio on right channel.")
            self.last_fall_event_time = current_time
        
        # 运动状态检测
        if acc_total_change < STILL_ACCEL_THRESHOLD and gyro_total_change < STILL_GYRO_THRESHOLD:
            new_state = 'still'
        else:
            new_state = 'moving'

        if new_state != self.movement_state:
            print(f"[INFO] Movement state changed from {self.movement_state} to {new_state}.")
            self.movement_state = new_state

    def process_xm125_data(self, data):
        distances = data.get('distances_m', [])
        strengths = data.get('strengths_db', [])
        if distances and strengths and len(distances) == len(strengths):
            # 直接获取最近的距离
            min_distance = distances[0]
            # print(f"[INFO] Nearest distance (XM125): {min_distance:.2f} m")

            if OBSTACLE_DISTANCE_MIN <= min_distance <= OBSTACLE_DISTANCE_THRESHOLD:
                freq = map_distance_to_frequency(min_distance)
                self.audio_player.frequency = freq  # 设置左声道频率
                # print(f"[DEBUG] Playing frequency sound at {freq:.2f} Hz on left channel.")
            else:
                # 停止声音
                self.audio_player.frequency = 0
                # print("[DEBUG] Obstacle out of range (XM125). Frequency sound stopped on left channel.")
        else:
            # 停止声音
            self.audio_player.frequency = 0
            # print("[DEBUG] No valid XM125 data detected. Frequency sound stopped on left channel.")


# ============================
# 文件事件处理类
# ============================

class DataFileHandler(FileSystemEventHandler):
    """
    处理文件修改事件，读取新数据并传递给数据处理器。
    """
    def __init__(self, data_processor):
        super().__init__()
        self.data_processor = data_processor
        try:
            self.file = open(DATA_FILE, 'r')
            # 移动到文件末尾
            self.file.seek(0, os.SEEK_END)
            print(f"[DEBUG] Opened data file '{DATA_FILE}' successfully.")
        except FileNotFoundError:
            print(f"[ERROR] Data file '{DATA_FILE}' not found.")
            self.file = None

    def on_modified(self, event):
        if self.file is None:
            return
        if event.src_path == os.path.abspath(DATA_FILE):
            self.process_new_data()

    def process_new_data(self):
        while True:
            line = self.file.readline()
            if not line:
                break
            try:
                data = json.loads(line)
                self.data_processor.process_data_entry(data)
            except json.JSONDecodeError:
                print("[DEBUG] Invalid JSON line encountered. Skipping.")
                continue  # 忽略错误的 JSON 行

# ============================
# 主函数
# ============================

def main():
    observer = None
    audio_player = None
    data_processor = None
    try:
        # 获取设备列表并打印
        devices = sd.query_devices()
        print("Available audio devices:")
        for idx, device in enumerate(devices):
            print(f"{idx}: {device['name']}")

        # 指定音频设备索引（根据您的设备选择）
        device_index = 0  # 例如，使用索引为 0 的设备

        # 初始化音频播放器
        audio_player = AudioPlayer(sample_rate=SAMPLE_RATE, device=device_index)
        audio_player.start()

        # 初始化数据处理器
        data_processor = SensorDataProcessor(audio_player)

        # 设置文件系统观察者
        event_handler = DataFileHandler(data_processor)
        observer = Observer()
        observer.schedule(event_handler, path=os.path.dirname(os.path.abspath(DATA_FILE)), recursive=False)
        observer.start()

        print("[INFO] Starting sensor data monitoring...")

        # 主线程保持运行
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                print("[INFO] Keyboard interrupt received. Exiting...")
                break

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        # 停止音频播放器
        if audio_player is not None:
            audio_player.stop()
        # 停止文件观察者
        if observer is not None:
            observer.stop()
            observer.join()
       # 关闭发送线程
        if data_processor is not None:
            data_processor.shutdown()
        print("[INFO] Audio player, observer, and data sender stopped. Resources have been released.")
if __name__ == "__main__":
    main()