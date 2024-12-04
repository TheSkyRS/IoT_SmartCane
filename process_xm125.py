#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XM125 传感器数据的简化处理算法，直接读取最近距离并实时调整声音频率
"""

import json
import numpy as np
import threading
import sounddevice as sd
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import time
import warnings
from scipy.io.wavfile import WavFileWarning

# 忽略 WavFileWarning 警告
warnings.filterwarnings("ignore", category=WavFileWarning)

# ============================
# 配置参数
# ============================

DATA_FILE = 'sensor_data.jsonl'

# XM125 阈值设置
OBSTACLE_DISTANCE_THRESHOLD = 0.9  # 米
OBSTACLE_DISTANCE_MIN = 0.1        # 米

# 频率映射
FREQ_MAX = 2000  # Hz
FREQ_MIN = 200   # Hz

# 声音参数
SAMPLE_RATE = 44100  # 采样率

# ============================
# 辅助函数和类
# ============================

def map_distance_to_frequency(distance):
    freq = FREQ_MIN + (OBSTACLE_DISTANCE_THRESHOLD - distance) * \
        (FREQ_MAX - FREQ_MIN) / (OBSTACLE_DISTANCE_THRESHOLD - OBSTACLE_DISTANCE_MIN)
    freq = np.clip(freq, FREQ_MIN, FREQ_MAX)
    return freq

class AudioPlayer:
    def __init__(self, sample_rate=44100, device=None):
        self.sample_rate = sample_rate
        self.frequency = 0
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

    def start(self):
        self.stream.start()

    def stop(self):
        self.stream.stop()
        self.stream.close()

    def set_frequency(self, frequency):
        with self.lock:
            self.frequency = frequency

    def audio_callback(self, outdata, frames, time_info, status):
        if status:
            print(status)
        t = (np.arange(frames) + self.phase) / self.sample_rate
        with self.lock:
            freq = self.frequency
        self.phase = (self.phase + frames) % self.sample_rate
        if freq > 0:
            outdata[:, 0] = np.sin(2 * np.pi * freq * t).astype(np.float32)
        else:
            outdata[:, 0].fill(0)

# ============================
# 数据处理类
# ============================

class XM125DataProcessor:
    def __init__(self, audio_player):
        self.audio_player = audio_player

    def process_xm125_data(self, data):
        distances = data.get('distances_m', [])
        strengths = data.get('strengths_db', [])
        if distances and strengths and len(distances) == len(strengths):
            # 直接获取最近的距离
            min_distance = min(distances)
            print(f"[INFO] Nearest distance: {min_distance:.2f} m")

            if OBSTACLE_DISTANCE_MIN <= min_distance <= OBSTACLE_DISTANCE_THRESHOLD:
                freq = map_distance_to_frequency(min_distance)
                self.audio_player.set_frequency(freq)
                print(f"[DEBUG] Playing sound at frequency {freq:.2f} Hz")
            else:
                # 停止声音
                self.audio_player.set_frequency(0)
                print("[DEBUG] Object out of range. Sound stopped.")
        else:
            # 停止声音
            self.audio_player.set_frequency(0)
            print("[DEBUG] No valid XM125 data detected. Sound stopped.")

    def process_data_entry(self, data):
        sensor_type = data.get('type')

        if sensor_type == 'XM125':
            self.process_xm125_data(data)

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

        # 初始化音频播放器
        audio_player = AudioPlayer(sample_rate=SAMPLE_RATE, device=device_index)
        audio_player.start()

        # 初始化数据处理器
        data_processor = XM125DataProcessor(audio_player)

        # 设置文件系统观察者
        event_handler = DataFileHandler(data_processor)
        observer = Observer()
        observer.schedule(event_handler, path=os.path.dirname(os.path.abspath(DATA_FILE)), recursive=False)
        observer.start()

        print("[INFO] Starting XM125 data monitoring...")

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