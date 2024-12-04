#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
低延迟、高精度的XM125距离检测算法
"""

import json
import time
import os
import numpy as np
import pygame
import threading

# ============================
# 配置参数
# ============================

DATA_FILE = 'sensor_data.jsonl'

# 阈值设置
OBSTACLE_DISTANCE_THRESHOLD = 0.7  # 米
OBSTACLE_DISTANCE_MIN = 0.1        # 米

# 频率映射
FREQ_MAX = 2000  # Hz
FREQ_MIN = 500   # Hz

# 声音持续时间
SOUND_DURATION = 0.05  # 秒

# 平滑因子
EMA_ALPHA = 0.5  # 平滑因子，取值在0到1之间，可根据需要调整

# 伪影距离阈值
OUTLIER_THRESHOLD = 0.5  # 米，当前测量值与EMA值的最大允许差异

# ============================
# 状态变量
# ============================

current_frequency = None
ema_distance = None  # EMA平滑后的距离

# 声音字典
sound_dict = {}

# 线程锁
lock = threading.Lock()

# ============================
# 初始化 Pygame mixer
# ============================

pygame.mixer.init(frequency=44100, size=-16, channels=2)
print("[INFO] Pygame mixer initialized.")

# ============================
# 辅助函数
# ============================

def generate_tone(frequency, duration=SOUND_DURATION):
    fs = 44100  # 采样率
    t = np.linspace(0, duration, int(fs * duration), False)
    tone = np.sin(frequency * t * 2 * np.pi)
    audio = tone * (2**15 - 1) / np.max(np.abs(tone))
    audio = audio.astype(np.int16)
    # 立体声
    stereo_audio = np.column_stack((audio, audio))
    sound = pygame.sndarray.make_sound(stereo_audio)
    return sound

def play_obstacle_sound(freq, OBSTACLE_CHANNEL):
    global current_frequency
    rounded_freq = round(freq)
    with lock:
        if current_frequency != rounded_freq:
            if OBSTACLE_CHANNEL.get_busy():
                OBSTACLE_CHANNEL.stop()
                print("[DEBUG] Stopped previous obstacle sound.")
            sound = sound_dict.get(rounded_freq)
            if not sound:
                sound = generate_tone(rounded_freq)
                sound_dict[rounded_freq] = sound
                print(f"[INFO] Generated new sound for frequency {rounded_freq} Hz.")
            OBSTACLE_CHANNEL.play(sound, loops=-1)
            current_frequency = rounded_freq
            print(f"[INFO] Playing obstacle sound at {rounded_freq} Hz.")
        else:
            print("[DEBUG] Current frequency unchanged. No need to update sound.")

def stop_obstacle_sound(OBSTACLE_CHANNEL):
    global current_frequency
    with lock:
        if OBSTACLE_CHANNEL.get_busy():
            OBSTACLE_CHANNEL.stop()
            print("[INFO] Stopped obstacle sound.")
        current_frequency = None

def process_frame(current_peaks, OBSTACLE_CHANNEL):
    global ema_distance

    if not current_peaks:
        stop_obstacle_sound(OBSTACLE_CHANNEL)
        return

    # 选择最近的峰值
    closest_peak = min(current_peaks, key=lambda x: x['distance'])
    current_distance = closest_peak['distance']
    current_strength = closest_peak['strength']

    # 初始化EMA
    if ema_distance is None:
        ema_distance = current_distance
    else:
        # 检查当前测量值与EMA值的差异
        if abs(current_distance - ema_distance) > OUTLIER_THRESHOLD:
            # 当前测量值可能是伪影，忽略
            print(f"[DEBUG] Outlier detected. Current distance: {current_distance:.2f} m, EMA distance: {ema_distance:.2f} m")
            current_distance = ema_distance  # 使用EMA值代替
        else:
            # 更新EMA值
            ema_distance = EMA_ALPHA * current_distance + (1 - EMA_ALPHA) * ema_distance

    print(f"[INFO] EMA distance: {ema_distance:.2f} m")

    # 检查距离是否在阈值范围内
    if OBSTACLE_DISTANCE_MIN <= ema_distance <= OBSTACLE_DISTANCE_THRESHOLD:
        freq = FREQ_MIN + (OBSTACLE_DISTANCE_THRESHOLD - ema_distance) * \
            (FREQ_MAX - FREQ_MIN) / (OBSTACLE_DISTANCE_THRESHOLD - OBSTACLE_DISTANCE_MIN)
        play_obstacle_sound(freq, OBSTACLE_CHANNEL)
    else:
        stop_obstacle_sound(OBSTACLE_CHANNEL)

def process_data_entry(data, OBSTACLE_CHANNEL):
    sensor_type = data.get('type')

    if sensor_type == 'XM125':
        distances = data.get('distances_m', [])
        strengths = data.get('strengths_db', [])
        if distances and strengths and len(distances) == len(strengths):
            current_peaks = [{'distance': d, 'strength': s} for d, s in zip(distances, strengths)]
            process_frame(current_peaks, OBSTACLE_CHANNEL)
        else:
            stop_obstacle_sound(OBSTACLE_CHANNEL)
            print("[INFO] XM125: No valid peaks detected. Stopping obstacle sound.")

def monitor_data_file():
    OBSTACLE_CHANNEL = pygame.mixer.Channel(0)
    print("[INFO] Sound channel allocated: Channel 0 for obstacles.")

    with open(DATA_FILE, 'r') as f:
        f.seek(0, os.SEEK_END)
        print(f"[INFO] Monitoring data file: {DATA_FILE}")

        while True:
            line = f.readline()
            if not line:
                time.sleep(0.01)  # 减少等待时间
                continue
            try:
                data = json.loads(line)
                process_data_entry(data, OBSTACLE_CHANNEL)
            except json.JSONDecodeError:
                print(f"[ERROR] Failed to decode JSON: {line.strip()}")
            except Exception as e:
                print(f"[ERROR] Error processing data: {e}")

def main():
    try:
        print("[INFO] Starting XM125 data monitoring...")
        monitor_data_file()
    except KeyboardInterrupt:
        print("\n[INFO] Detection stopped by user.")
    finally:
        pygame.mixer.quit()
        print("[INFO] Pygame mixer quit. Resources have been released.")

if __name__ == "__main__":
    main()