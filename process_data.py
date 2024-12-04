#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
处理 HC_SR04 和 MPU6050 传感器数据的实时算法，包括事件检测和音频提示
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

# 忽略 WavFileWarning 警告
warnings.filterwarnings("ignore", category=WavFileWarning)

# ============================
# 配置参数
# ============================

DATA_FILE = 'sensor_data.jsonl'

# HC_SR04 阈值设置
UP_STEP_DISTANCE_THRESHOLD = 0.3   # 米，小于此距离认为是上台阶
DOWN_STEP_DISTANCE_THRESHOLD = 0.5  # 米，大于此距离认为是下台阶
HC_SR04_DEBOUNCE_TIME = 1.0        # 秒，防抖时间
HC_SR04_COOLDOWN_TIME = 3.0        # 秒，提示音冷却时间

# MPU6050 阈值设置
FALL_ACCEL_THRESHOLD = 2.5         # g，加速度阈值，超过此值认为是摔倒
FALL_GYRO_THRESHOLD = 300          # °/s，角速度阈值
FALL_DEBOUNCE_TIME = 2.0           # 秒，防抖时间
FALL_COOLDOWN_TIME = 5.0           # 秒，提示音冷却时间

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

# ============================
# 辅助函数和类
# ============================

class AudioPlayer:
    def __init__(self, sample_rate=44100, device=None):
        self.sample_rate = sample_rate
        self.phase = 0
        self.lock = threading.Lock()
        self.device = device
        self.stream = sd.OutputStream(
            channels=1,
            callback=self.audio_callback,
            samplerate=self.sample_rate,
            device=self.device,
            blocksize=0,
            dtype='float32'
        )
        self.alert_lock = threading.Lock()
        self.preloaded_sounds = {}  # 预加载的音频数据
        self.alert_queue = queue.PriorityQueue()
        self.current_alert_sound = None
        self.alert_sound_position = 0
        self.alerts_in_queue = set()  # 记录已在队列中的警告音

        # 预加载警告音频
        for sound_file in [UP_AUDIO_FILE, DOWN_AUDIO_FILE, FALL_AUDIO_FILE]:
            samplerate, data = wavfile.read(sound_file)
            data = data.astype(np.float32) / np.iinfo(data.dtype).max  # 归一化数据
            if data.ndim > 1 and data.shape[1] > 1:
                data = data.mean(axis=1)  # 将立体声转换为单声道
            self.preloaded_sounds[sound_file] = data

    def start(self):
        self.stream.start()

    def stop(self):
        self.stream.stop()
        self.stream.close()

    def play_alert_sound(self, sound_file):
        priority = ALERT_PRIORITIES.get(sound_file, 10)  # 默认优先级为 10
        with self.alert_lock:
            if sound_file not in self.alerts_in_queue:
                self.alert_queue.put((priority, sound_file))
                self.alerts_in_queue.add(sound_file)
                print(f"[DEBUG] Added '{sound_file}' to alert queue.")

    def audio_callback(self, outdata, frames, time_info, status):
        if status:
            print(status)

        with self.alert_lock:
            if self.current_alert_sound is None and not self.alert_queue.empty():
                # 获取优先级最高的警告音
                _, sound_file = self.alert_queue.get()
                self.current_alert_sound = self.preloaded_sounds.get(sound_file)
                self.alert_sound_position = 0
                self.alerts_in_queue.discard(sound_file)
                print(f"[DEBUG] Now playing '{sound_file}'.")

            if self.current_alert_sound is not None:
                data = self.current_alert_sound
                start = self.alert_sound_position
                end = start + frames
                chunk = data[start:end]
                if len(chunk) < frames:
                    # 警告音播放完毕，填充剩余部分
                    outdata[:len(chunk), 0] = chunk
                    outdata[len(chunk):, 0].fill(0)
                    self.current_alert_sound = None
                else:
                    outdata[:, 0] = chunk
                    self.alert_sound_position += frames
            else:
                # 没有警告音，静音
                outdata[:, 0].fill(0)

# ============================
# 数据处理类
# ============================

class SensorDataProcessor:
    def __init__(self, audio_player):
        self.audio_player = audio_player
        self.last_hc_sr04_event_time = 0
        self.last_fall_event_time = 0
        self.last_hc_sr04_state = 'unknown'  # 初始状态为未知
        self.last_alert_times = {
            UP_AUDIO_FILE: 0,
            DOWN_AUDIO_FILE: 0,
            FALL_AUDIO_FILE: 0
        }

    def process_hc_sr04_data(self, data):
        current_time = time.time()
        distance = data.get('distance_m')
        if distance is None:
            return

        # 打印距离信息
        print(f"[DEBUG] HC_SR04 distance: {distance:.2f} meters")

        # 防抖处理
        if current_time - self.last_hc_sr04_event_time < HC_SR04_DEBOUNCE_TIME:
            return

        # 确定当前状态
        if distance < UP_STEP_DISTANCE_THRESHOLD:
            current_state = 'up'
        elif distance > DOWN_STEP_DISTANCE_THRESHOLD:
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
                    print("[INFO] Detected upward step. Playing 'up' audio.")
            elif current_state == 'down':
                # 冷却时间检查
                if current_time - self.last_alert_times[DOWN_AUDIO_FILE] >= HC_SR04_COOLDOWN_TIME:
                    # 播放下楼梯提示音
                    self.audio_player.play_alert_sound(DOWN_AUDIO_FILE)
                    self.last_alert_times[DOWN_AUDIO_FILE] = current_time
                    print("[INFO] Detected downward step. Playing 'down' audio.")

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
        print(f"[DEBUG] MPU6050 Acc Total Change: {acc_total_change:.2f} g")
        print(f"[DEBUG] MPU6050 Gyro Total Change: {gyro_total_change:.2f} °/s")

        if acc_total_change >= FALL_ACCEL_THRESHOLD or gyro_total_change >= FALL_GYRO_THRESHOLD:
            # 防抖处理
            if current_time - self.last_fall_event_time < FALL_DEBOUNCE_TIME:
                return
            # 冷却时间检查
            if current_time - self.last_alert_times[FALL_AUDIO_FILE] >= FALL_COOLDOWN_TIME:
                # 检测到摔倒，播放提示音
                self.audio_player.play_alert_sound(FALL_AUDIO_FILE)
                self.last_alert_times[FALL_AUDIO_FILE] = current_time
                print("[INFO] Fall detected. Playing 'fall' audio.")
            self.last_fall_event_time = current_time

    def process_data_entry(self, data):
        sensor_type = data.get('type')

        if sensor_type == 'HC_SR04':
            self.process_hc_sr04_data(data)
        elif sensor_type == 'MPU6050':
            self.process_mpu6050_data(data)

# ============================
# 文件事件处理类
# ============================

class DataFileHandler(FileSystemEventHandler):
    def __init__(self, data_processor):
        super().__init__()
        self.data_processor = data_processor
        self.file = open(DATA_FILE, 'r')
        # 移动到文件末尾
        self.file.seek(0, os.SEEK_END)

    def on_modified(self, event):
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
                continue  # 忽略错误的 JSON 行

# ============================
# 主函数
# ============================

def main():
    observer = None
    audio_player = None
    try:
        # 获取设备列表并打印
        devices = sd.query_devices()
        print("Available audio devices:")
        for idx, device in enumerate(devices):
            print(f"{idx}: {device['name']}")

        # 指定音频设备索引（根据您的设备选择）
        device_index = 0  # 例如，使用索引为 0 的设备

        # 检查音频文件是否存在
        for audio_file in [UP_AUDIO_FILE, DOWN_AUDIO_FILE, FALL_AUDIO_FILE]:
            if not os.path.exists(audio_file):
                raise FileNotFoundError(f"Audio file '{audio_file}' not found.")

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

        print("[INFO] Starting HC_SR04 and MPU6050 data monitoring...")

        # 主线程保持运行
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
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
        print("[INFO] Audio player and observer stopped. Resources have been released.")

if __name__ == "__main__":
    main()