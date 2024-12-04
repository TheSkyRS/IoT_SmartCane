#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高精度、低延迟的XM125距离检测算法，使用Kalman滤波器、多线程和异步I/O
"""

import json
import time
import os
import numpy as np
import pygame
import threading
import asyncio
from scipy.stats import norm

# ============================
# 配置参数
# ============================

DATA_FILE = 'sensor_data.jsonl'

# 阈值设置
OBSTACLE_DISTANCE_THRESHOLD = 0.9  # 米
OBSTACLE_DISTANCE_MIN = 0.1        # 米

CONFIDENCE_THRESHOLD = 2
CONFIDENCE_MAX = 3

# 频率映射
FREQ_MAX = 2000  # Hz
FREQ_MIN = 500   # Hz

# Kalman滤波器参数
KALMAN_PROCESS_NOISE = 1e-2  # 过程噪声协方差
KALMAN_MEASUREMENT_NOISE = 1e-1  # 测量噪声协方差
KALMAN_ERROR_COVARIANCE = 1.0  # 估计误差协方差初始值

# 物体匹配阈值
DISTANCE_MATCH_THRESHOLD = 0.15  # 米

# 声音参数
SAMPLE_RATE = 44100  # 采样率
CHUNK_SIZE = 1024    # 音频块大小

# ============================
# 状态变量
# ============================

current_frequency = None
object_tracks = {}  # 存储活动的物体轨迹
next_object_id = 1  # 分配给下一个新物体的ID

# 线程锁
lock = threading.Lock()

# 事件循环
event_loop = asyncio.new_event_loop()

# ============================
# 初始化 Pygame mixer
# ============================

pygame.mixer.init(frequency=SAMPLE_RATE, size=-16, channels=1)
print("[INFO] Pygame mixer initialized.")

# ============================
# 辅助函数
# ============================

class KalmanFilter1D:
    def __init__(self, process_noise, measurement_noise, error_covariance, initial_state):
        self.process_noise = process_noise  # Q
        self.measurement_noise = measurement_noise  # R
        self.error_covariance = error_covariance  # P
        self.state_estimate = initial_state  # x

    def update(self, measurement):
        # 预测阶段
        self.error_covariance += self.process_noise

        # 更新阶段
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_noise)
        self.state_estimate += kalman_gain * (measurement - self.state_estimate)
        self.error_covariance = (1 - kalman_gain) * self.error_covariance

        return self.state_estimate

def generate_tone(frequency, duration):
    t = np.linspace(0, duration, int(SAMPLE_RATE * duration), False)
    tone = np.sin(frequency * t * 2 * np.pi)
    audio = tone * 32767  # 按16位PCM格式
    audio = audio.astype(np.int16)
    return audio

async def play_tone_stream(frequency, stop_event):
    """
    实时播放指定频率的音调，直到 stop_event 被设置。
    """
    stream = pygame.mixer.Sound(buffer=generate_tone(frequency, 0.1)).play(-1)
    while not stop_event.is_set():
        await asyncio.sleep(0.01)
    stream.stop()

def map_distance_to_frequency(distance):
    freq = FREQ_MIN + (OBSTACLE_DISTANCE_THRESHOLD - distance) * \
        (FREQ_MAX - FREQ_MIN) / (OBSTACLE_DISTANCE_THRESHOLD - OBSTACLE_DISTANCE_MIN)
    freq = np.clip(freq, FREQ_MIN, FREQ_MAX)
    return freq

# 在物体匹配过程中，增加置信度处理
def process_frame(current_peaks):
    global object_tracks, next_object_id

    current_peaks = [{'distance': d, 'strength': s} for d, s in current_peaks]
    new_object_tracks = {}
    matched_objects = set()

    for peak in current_peaks:
        matched = False
        for obj_id, obj_info in object_tracks.items():
            distance_diff = abs(obj_info['distance'] - peak['distance'])
            if distance_diff <= DISTANCE_MATCH_THRESHOLD:
                # 匹配成功，更新Kalman滤波器
                estimated_distance = obj_info['kalman_filter'].update(peak['distance'])
                new_object_tracks[obj_id] = {
                    'distance': estimated_distance,
                    'kalman_filter': obj_info['kalman_filter'],
                    'confidence': min(obj_info['confidence'] + 1, CONFIDENCE_MAX)  # 增加置信度
                }
                matched_objects.add(obj_id)
                matched = True
                break
        if not matched:
            # 新物体，创建Kalman滤波器
            kf = KalmanFilter1D(
                process_noise=KALMAN_PROCESS_NOISE,
                measurement_noise=KALMAN_MEASUREMENT_NOISE,
                error_covariance=KALMAN_ERROR_COVARIANCE,
                initial_state=peak['distance']
            )
            new_object_tracks[next_object_id] = {
                'distance': peak['distance'],
                'kalman_filter': kf,
                'confidence': 1  # 初始置信度为1
            }
            next_object_id += 1

    # 对于未匹配的物体，减少置信度
    for obj_id, obj_info in object_tracks.items():
        if obj_id not in matched_objects:
            obj_info['confidence'] -= 1  # 减少置信度
            if obj_info['confidence'] > 0:
                new_object_tracks[obj_id] = obj_info
            else:
                print(f"[DEBUG] Object {obj_id} removed due to low confidence.")

    object_tracks = new_object_tracks

    # 选择置信度足够高的物体
    valid_objects = {obj_id: obj_info for obj_id, obj_info in object_tracks.items() if obj_info['confidence'] >= CONFIDENCE_THRESHOLD}

    if valid_objects:
        # 选择最近的物体
        closest_obj = min(valid_objects.values(), key=lambda x: x['distance'])
        closest_distance = closest_obj['distance']
        print(f"[INFO] Estimated distance: {closest_distance:.2f} m (Confidence: {closest_obj['confidence']})")

        if OBSTACLE_DISTANCE_MIN <= closest_distance <= OBSTACLE_DISTANCE_THRESHOLD:
            freq = map_distance_to_frequency(closest_distance)
            # 在事件循环中调用异步播放函数
            asyncio.run_coroutine_threadsafe(update_sound(freq), event_loop)
        else:
            # 停止声音
            asyncio.run_coroutine_threadsafe(stop_sound(), event_loop)
    else:
        # 没有置信度足够高的物体，停止声音
        asyncio.run_coroutine_threadsafe(stop_sound(), event_loop)

# 声音控制变量
current_sound_task = None
sound_stop_event = threading.Event()

async def update_sound(frequency):
    global current_frequency, current_sound_task, sound_stop_event

    if current_frequency != frequency:
        # 如果正在播放声音，先停止
        if current_sound_task and not current_sound_task.done():
            sound_stop_event.set()
            await current_sound_task
            print("[DEBUG] Stopped previous sound.")
        # 重置停止事件
        sound_stop_event.clear()
        # 启动新的声音任务
        current_sound_task = asyncio.create_task(play_tone_stream(frequency, sound_stop_event))
        current_frequency = frequency
        print(f"[INFO] Playing sound at {frequency:.2f} Hz.")
    else:
        print("[DEBUG] Sound frequency unchanged.")

async def stop_sound():
    global current_sound_task, current_frequency, sound_stop_event

    if current_sound_task and not current_sound_task.done():
        sound_stop_event.set()
        await current_sound_task
        print("[INFO] Sound stopped.")
    current_sound_task = None
    current_frequency = None

def process_data_entry(data):
    sensor_type = data.get('type')

    if sensor_type == 'XM125':
        distances = data.get('distances_m', [])
        strengths = data.get('strengths_db', [])
        if distances and strengths and len(distances) == len(strengths):
            current_peaks = list(zip(distances, strengths))
            process_frame(current_peaks)
        else:
            # 停止声音
            asyncio.run_coroutine_threadsafe(stop_sound(), event_loop)
            print("[INFO] XM125: No valid peaks detected. Stopping sound.")

async def monitor_data_file():
    print("[INFO] Monitoring data file: {}".format(DATA_FILE))

    # 异步打开文件
    loop = asyncio.get_event_loop()
    with open(DATA_FILE, 'r') as f:
        # 移动到文件末尾
        f.seek(0, os.SEEK_END)

        while True:
            line = f.readline()
            if not line:
                await asyncio.sleep(0.01)
                continue
            try:
                data = json.loads(line)
                # 在线程池中处理数据
                await loop.run_in_executor(None, process_data_entry, data)
            except json.JSONDecodeError:
                print(f"[ERROR] Failed to decode JSON: {line.strip()}")
            except Exception as e:
                print(f"[ERROR] Error processing data: {e}")

def main():
    global event_loop
    try:
        print("[INFO] Starting XM125 data monitoring...")

        # 在单独的线程中运行事件循环
        def start_event_loop(loop):
            asyncio.set_event_loop(loop)
            loop.run_forever()

        threading.Thread(target=start_event_loop, args=(event_loop,), daemon=True).start()

        # 运行数据监控协程
        asyncio.run(monitor_data_file())
    except KeyboardInterrupt:
        print("\n[INFO] Detection stopped by user.")
    finally:
        # 关闭事件循环
        event_loop.call_soon_threadsafe(event_loop.stop)
        pygame.mixer.quit()
        print("[INFO] Pygame mixer quit. Resources have been released.")

if __name__ == "__main__":
    main()