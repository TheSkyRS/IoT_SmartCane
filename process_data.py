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
import io
import pyttsx3
import whisper
import fastapi_poe as fp
import asyncio

system_prompt = """
You are a voice assistant for a smart cane with various functionalities. Interpret the user's commands and respond with a JSON object without any markdown. Use the following format: { "name": "<function_name>", "args": [<arguments>] }.

Sensors:
- XM125: Pulsed Coherent Radar Sensor to detect obstacle distance.
- HC_SR04: Ultrasonic Sensor to detect distance.
- MPU6050: 6-DoF Accel and Gyro Sensor (also called IMU).

Available functions:
- Set XM125 Threshold: { "name": "set_xm125", "args": [<min_distance>, <max_distance>] }
- Set HC_SR04 Threshold: { "name": "set_hs_sr04", "args": [<up_threshold>, <down_threshold>] }
- Display Text: { "name": "display_text", "args": [ "<text>" ] }

Instructions:
1. Understand the user's request or question.
2. Attempt to parse it into one of the available functions and return the corresponding JSON format.
3. If the request does not fit any of the above functions, answer the question based on your own knowledge and use the "display_text" function to provide your response in the following format: { "name": "display_text", "args": [ "Your response text here." ] }.

Ensure all responses are in JSON format and provided as a single-line string without line breaks.

Note:
- If the user sets only one of the arguments for a function, set the other argument to null. For example, if the user calls "set XM125 Threshold min to 0.2m", then return { "name": "set_xm125", "args": [0.2, null] }.
"""




# 忽略 WavFileWarning 警告
warnings.filterwarnings("ignore", category=WavFileWarning)

# ============================
# 配置参数
# ============================

DATA_FILE = 'sensor_data.jsonl'

# HC_SR04 阈值设置
UP_STEP_DISTANCE_THRESHOLD = 0.15     # 米，小于此距离认为是上台阶
DOWN_STEP_DISTANCE_THRESHOLD = 0.5   # 米，大于此距离认为是下台阶
HC_SR04_DEBOUNCE_TIME = 1.0          # 秒，防抖时间
HC_SR04_COOLDOWN_TIME = 3.0          # 秒，提示音冷却时间
HC_SR04_BUFFER_SIZE = 5
HC_SR04_MAX_THRESHOLD = 2 # 米
HC_SR04_MIN_THRESHOLD = 0.05 # 米

# MPU6050 阈值设置
FALL_ACCEL_THRESHOLD = 1.5           # g，加速度阈值，超过此值认为是摔倒
ROTATION_GYRO_THRESHOLD = 300 # 开启语音助手
FALL_DEBOUNCE_TIME = 3.0             # 秒，防抖时间
FALL_COOLDOWN_TIME = 10.0             # 秒，提示音冷却时间

POE_API_KEY = '2-j_yibA8pbfozuVugcd8qTYAC-4Iu4XHNEOtuEV5qE'

RECORDING_DURATION = 5 # 语音助手录音

STILL_ACCEL_THRESHOLD = 0.1  # g
STILL_GYRO_THRESHOLD = 10.0  # °/s

MOVING_COUNT = 5

# XM125 阈值设置
OBSTACLE_DISTANCE_THRESHOLD = 1.2    # 米
OBSTACLE_DISTANCE_MIN = 0.1          # 米
XM125_MAX_LIMIT = 3 # Profile 1 Limit

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

CLOUD_SERVER_URL = 'http://34.72.243.54:5000'
BATCH_SIZE = 10
BATCH_TIMEOUT = 3
QUEUE_SIZE = 1000
NTFY_URL = 'http://ntfy.sh/SmartCane'

# ============================
# 辅助函数和类
# ============================

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

        # 新增：静音控制
        self.muted = False
        self.mute_lock = threading.Lock()
        # 初始化 TTS 引擎
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # 设置语速

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

    def estimate_tts_duration(self, text):
        """
        根据文本长度估算 TTS 播放所需时间（秒）。
        """
        words = len(text.split())
        words_per_minute = self.tts_engine.getProperty('rate')  # 获取语速
        words_per_minute = max(words_per_minute, 1)  # 防止除零错误
    
        duration_seconds = (words / words_per_minute) * 60
        return duration_seconds

    def speak_text(self, text):
        """
        将文本转换为语音并播放。在播放期间暂停所有音频输出，播放结束后恢复。
        """
        print(f"[INFO] Speaking text: {text}")

        # 计算预计播放时间
        duration = self.estimate_tts_duration(text)
        print(f"[DEBUG] Estimated TTS duration: {duration:.2f} seconds")

        # 设置静音
        with self.mute_lock:
            self.muted = True

        # 开始 TTS 播放
        def tts_thread():
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()

        tts = threading.Thread(target=tts_thread, daemon=True)
        tts.start()

        # 设置定时器在预计播放时间后取消静音
        def unmute():
            with self.mute_lock:
                self.muted = False
            print("[DEBUG] Audio unmuted after TTS.")

        timer = threading.Timer(duration, unmute)
        timer.start()

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

        # 检查是否静音
        with self.mute_lock:
            if self.muted:
                outdata.fill(0)
                return

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

        self.xm125_min = OBSTACLE_DISTANCE_MIN
        self.xm125_max = OBSTACLE_DISTANCE_THRESHOLD
        self.hc_sr04_up = UP_STEP_DISTANCE_THRESHOLD
        self.hc_sr04_down = DOWN_STEP_DISTANCE_THRESHOLD

        # 新增：状态管理
        self.state = 'normal'  # normal fall recording communicating
        self.state_lock = threading.Lock()
        # 新增：用于跟踪连续的 'moving' 状态检测次数
        self.moving_state_counter = 0
        self.required_consecutive_movings = MOVING_COUNT  # 连续检测次数

        # 新增：用于标记音频上传完成的事件
        self.audio_upload_done = threading.Event()
        self.audio_upload_done.set()  # 初始为已完成

         # 加载 Whisper 模型
        self.whisper_model = whisper.load_model("base.en")
        
        # 发送队列和线程初始化
        self.send_queue = queue.Queue(maxsize=QUEUE_SIZE)
        self.batch_size = BATCH_SIZE  # 每批发送的数据量
        self.batch_timeout = BATCH_TIMEOUT  # 最大等待时间（秒）发送数据，即使未达到 batch_size
        self.sender_thread = threading.Thread(target=self._send_worker, daemon=True)
        self.sender_thread.start()

    def map_distance_to_frequency(self, distance):
        """
        将障碍物距离映射到频率范围内。
        """
        freq = FREQ_MIN + (self.xm125_max - distance) * \
            (FREQ_MAX - FREQ_MIN) / (self.xm125_max - self.xm125_min)
        freq = np.clip(freq, FREQ_MIN, FREQ_MAX)
        return freq
   # 新增：录音并使用 Whisper 进行转文字
    def start_voice_assistant_recording(self):
        with self.state_lock:
            if self.state != 'normal':
                print("[WARNING] Voice assistant is not in a state to start recording.")
                return
            self.state = 'recording'
            print("[INFO] Starting voice assistant recording.")
            self.audio_player.speak_text("Smart Assistant.")
            # 开始录音线程
            threading.Thread(target=self.record_audio, daemon=True).start()

    def stop_voice_assistant_recording(self):
        with self.state_lock:
            if self.state != 'recording':
                print("[WARNING] Voice assistant is not currently recording.")
                return
            self.state = 'communicating'
            print("[INFO] Stopping voice assistant recording.")
            self.audio_player.speak_text("Communicating with assistant.")
            # 停止录音并处理
            threading.Thread(target=self.process_recorded_audio, daemon=True).start()

    def record_audio(self):
        """
        录制用户的语音命令。
        """
        try:
            # 防止录音到“Smart Assistant.”
            time.sleep(1)
            print("[INFO] Recording audio...")
            audio = sd.rec(
                int(RECORDING_DURATION * SAMPLE_RATE),
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype='float32'
            )
            sd.wait()  # 等待录制完成
            wavfile.write('assistant_recording.wav', SAMPLE_RATE, audio)
            print("[INFO] Recording completed.")
            # 自动停止录音
            self.stop_voice_assistant_recording()
        except Exception as e:
            print(f"[ERROR] Failed to record audio: {e}")
            with self.state_lock:
                self.state = 'normal'


    def process_recorded_audio(self):
        """
        使用 Whisper 将录制的音频转为文字，并与 NLP 模型交互。
        """
        try:
            print("[INFO] Transcribing audio with Whisper...")
            result = self.whisper_model.transcribe("assistant_recording.wav")
            text = result['text'].strip()
            print(f"[INFO] Transcribed Text: {text}")
            if text:
                self.send_text_to_nlp(text)
            else:
                print("[ERROR] No speech detected in the recording.")
                with self.state_lock:
                    self.state = 'normal'
        except Exception as e:
            print(f"[ERROR] Failed to transcribe audio: {e}")
            with self.state_lock:
                self.state = 'normal'
        finally:
            # 清理录音文件
            if os.path.exists("assistant_recording.wav"):
                os.remove("assistant_recording.wav")

    async def get_llm_response(self, prompt):
        combined_prompt = f"{system_prompt}\nUser request: {prompt}"
        message = fp.ProtocolMessage(role="user", content=combined_prompt)
        full_response = ""

        async for partial in fp.get_bot_response(
            messages=[message],
            bot_name='GPT-4o-Mini',
            api_key=POE_API_KEY
        ):
            full_response += partial.text

        return full_response

    def send_text_to_nlp(self, text):
        """
        发送转录的文字到 NLP 模型并处理响应。
        """
        try:
            print("[INFO] Sending text to NLP model...")
            full_response = asyncio.run(self.get_llm_response(text))
            print(f"[INFO] NLP Response: {full_response}")
            # 处理 NLP 响应
            self.handle_nlp_response(full_response)
        except Exception as e:
            print(f"[ERROR] Failed to get LLM response: {e}")
            with self.state_lock:
                self.state = 'normal'

    def handle_nlp_response(self, response):
        """
        根据 NLP 模型的响应执行相应的功能，并将 display_text 转为音频输出。
        """
        try:
            parsed_response = json.loads(response)
            print(f"[INFO] Parsed NLP Response: {parsed_response}")
            # 根据响应执行相应的功能
            if 'name' in parsed_response and 'args' in parsed_response:
                function_name = parsed_response['name']
                args = parsed_response['args']
                
                if function_name == 'display_text':
                    text_to_display = args[0]
                    print(f"[DISPLAY] {text_to_display}")
                    # 将 display_text 转为音频输出
                    self.audio_player.speak_text(text_to_display)
                
                elif function_name == 'set_xm125':
                    self.xm125_min = args[0] if args[0] is not None and OBSTACLE_DISTANCE_MIN <= args[0] <= XM125_MAX_LIMIT else self.xm125_min
                    self.xm125_max = args[1] if args[1] is not None and OBSTACLE_DISTANCE_MIN <= args[1] <= XM125_MAX_LIMIT else self.xm125_max
                    self.audio_player.speak_text(f'Radar Threshold set min {self.xm125_min} meter max {self.xm125_max} meter')
                    print(f"[INFO] XM125 Threshold set to min: {self.xm125_min}m, max: {self.xm125_max}m.")
                
                elif function_name == 'set_hs_sr04':
                    self.hc_sr04_up = args[0] if args[0] is not None and HC_SR04_MIN_THRESHOLD <= args[0] <= HC_SR04_MAX_THRESHOLD else self.hc_sr04_up
                    self.hc_sr04_down = args[1] if args[1] is not None and HC_SR04_MIN_THRESHOLD <= args[1] <= HC_SR04_MAX_THRESHOLD else self.hc_sr04_down
                    self.audio_player.speak_text(f'Ultrasonic Threshold set up {self.hc_sr04_up} meter down {self.hc_sr04_down} meter')
                    print(f"[INFO] HC_SR04 Threshold set to up: {self.hc_sr04_up}m, down: {self.hc_sr04_down}m.")
                
                else:
                    print(f"[ERROR] Unknown function: {function_name}")
            else:
                print("[ERROR] Invalid response structure.")
        
        except (json.JSONDecodeError, KeyError) as e:
            print(f"[ERROR] Invalid NLP response format: {e}")
        
        finally:
            # 将状态切换回 'normal'
            with self.state_lock:
                self.state = 'normal'
                print("[INFO] State changed to 'normal'.")
   
    # 新增：发送 ntfy 通知的方法
    def send_ntfy_notification(self):
        try:
            ntfy_url = NTFY_URL
            message = "Fall Detection!!! Recording audio.\nGo to webstie: http://34.72.243.54:5000/fall_audios"
            response = requests.post(ntfy_url, data=message.encode('utf-8'))
            response.raise_for_status()
            print("[INFO] Ntfy Send.")
        except requests.exceptions.RequestException as e:
            print(f"[ERROR] Failed to send Ntfy: {e}")

    # 新增：录制音频并上传到云端的方法
    def record_and_send_audio(self):
        try:
            print("[INFO] Recording 10s ......")
            audio = sd.rec(
            int(FALL_COOLDOWN_TIME * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='float32'
            )
            sd.wait()  # 等待录制完成
            print("[INFO] Record Done, sending to GCP.")
            
            # 将录制的音频保存到内存缓冲区
            buffer = io.BytesIO()
            wavfile.write(buffer, SAMPLE_RATE, audio)
            buffer.seek(0)

            # 上传音频到云端
            files = {'file': ('fall_audio.wav', buffer, 'audio/wav')}
            response = requests.post(CLOUD_SERVER_URL+'/audio', files=files, timeout=BATCH_TIMEOUT)
            response.raise_for_status()
            print("[INFO] Send audio to GCP.")
        except Exception as e:
            print(f"[ERROR] Failed to send audio to GCP: {e}")
        finally:
            # 标记音频上传完成
            self.audio_upload_done.set()
    
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
            response = requests.post(CLOUD_SERVER_URL+'/data', json=data_pack, timeout=BATCH_TIMEOUT)
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
        with self.state_lock:
            if self.state == 'fall' or self.state == 'recording':
                return
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
        # print(f"[DEBUG] HC_SR04 median distance: {median_distance:.2f} meters")
        # self.hc_sr04_distance_buffer.clear()

        # 防抖处理
        if current_time - self.last_hc_sr04_event_time < HC_SR04_DEBOUNCE_TIME:
            # print("[DEBUG] HC_SR04 event ignored due to debounce.")
            return

        # 确定当前状态
        if median_distance <= self.hc_sr04_up:
            current_state = 'up'
        elif self.hc_sr04_down < median_distance <= HC_SR04_MAX_THRESHOLD:
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

        if gyro_total_change >= ROTATION_GYRO_THRESHOLD:
            # 防止多次触发，我们希望的是得到回应后，等语音讲完才能触发下次功能.
            with self.audio_player.mute_lock:
                if self.audio_player.muted:
                    return
            
            self.start_voice_assistant_recording()

        if acc_total_change >= FALL_ACCEL_THRESHOLD:
            # 防止SmartAssistant讲话时触发Fall.
            with self.audio_player.mute_lock:
                if self.audio_player.muted:
                    return
            
            # 防抖处理
            if current_time - self.last_fall_event_time < FALL_DEBOUNCE_TIME:
                # print("[DEBUG] MPU6050 fall event ignored due to debounce.")
                return
            # 冷却时间检查
            if current_time - self.last_alert_times[FALL_AUDIO_FILE] >= FALL_COOLDOWN_TIME:
                with self.state_lock:
                    if self.state == 'normal':
                        self.state = 'fall'
                        # 检测到摔倒，播放提示音
                        self.audio_player.play_alert_sound(FALL_AUDIO_FILE)
                        self.last_alert_times[FALL_AUDIO_FILE] = current_time
                        self.audio_upload_done.clear()  # 重置事件
                        print("[INFO] Fall detected.")                    
                        threading.Thread(target=self.send_ntfy_notification).start()
                        threading.Thread(target=self.record_and_send_audio).start()
            self.last_fall_event_time = current_time
        
        # 运动状态检测
        if acc_total_change < STILL_ACCEL_THRESHOLD and gyro_total_change < STILL_GYRO_THRESHOLD:
            new_state = 'still'
        else:
            new_state = 'moving'


        if new_state != self.movement_state:
            print(f"[INFO] Movement state changed from {self.movement_state} to {new_state}.")
            self.movement_state = new_state

        # 如果当前在 fall 状态，处理状态恢复逻辑
        with self.state_lock:
            if self.state == 'fall':
                if self.audio_upload_done.is_set():  # 仅当音频上传完成时处理恢复逻辑
                    if new_state == 'moving':
                        self.moving_state_counter += 1
                        print(f"[DEBUG] moving counter: {self.moving_state_counter}")
                        if self.moving_state_counter >= self.required_consecutive_movings:
                            self.state = 'normal'
                            self.moving_state_counter = 0
                            print("[INFO] From fall to normal state.")
                    else:
                        self.moving_state_counter = 0
                        print("[DEBUG] no moving, moving counter clear to 0.")
    def process_xm125_data(self, data):
        # with self.state_lock:
        if self.state == 'fall' or self.state == 'recording':
            self.audio_player.frequency = 0 # 停止声音
            return
        distances = data.get('distances_m', [])
        strengths = data.get('strengths_db', [])
        if distances and strengths and len(distances) == len(strengths):
            # 直接获取最近的距离
            min_distance = distances[0]
            # print(f"[INFO] Nearest distance (XM125): {min_distance:.2f} m")

            if self.xm125_min <= min_distance <= self.xm125_max:
                freq = self.map_distance_to_frequency(min_distance)
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