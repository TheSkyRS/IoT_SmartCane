# Intelligent Cane Project

This project is an intelligent cane designed to help visually impaired individuals navigate the world with confidence. The cane integrates a Raspberry Pi, microphone, speaker, and three sensors: the HCSR04 ultrasonic distance sensor, XM125 Pulse Coherent Radar millimeter-wave sensor (for obstacle distance measurement), and MPU6050 6-DoF Accelerometer and Gyro (also called IMU).

### Features and Functions:

#### 1. **HCSR04 Ultrasonic Sensor**:
- Detects changes in step height to prevent the user from tripping or missing a step.
- When the sensor detects a decrease in distance, the speaker announces "Upstairs."
- When the sensor detects an increase in distance, the speaker announces "Downstairs."

#### 2. **XM125 Radar Sensor**:
- Detects obstacles in the forward area and returns distance data.
- When obstacles enter the detection range, the speaker emits a single-frequency tone.
- The tone's frequency increases as the distance to the obstacle decreases, providing precise proximity feedback.
- Single-frequency sounds are chosen for their high sensitivity among visually impaired individuals and their distinctiveness in noisy environments.

#### 3. **MPU6050 IMU**:
- Tracks the cane's motion state to determine if the user is walking or stationary.
- Step detection using HCSR04 is only active when the cane is stationary to avoid false positives caused by the sensor's physical limitations.
- Detects falls and triggers two actions:
  1. Sends a notification (text message) to the user's family via Ntfy.
  2. Uploads a 10-second audio recording of the environment to GCP, accessible via a website.
- During audio recording, XM125 and HCSR04 sound alerts are paused for clear audio capture, aiding the family in understanding the user's surroundings.

#### 4. **Voice Assistant**:
- Activated by gently rotating the cane.
- Allows users to communicate with the voice assistant directly.
- During interactions with the large language model, XM125 and HCSR04 sound alerts are paused for clear microphone input.
- Converts the user's speech to text for interaction with the language model. While awaiting the model's response, normal sensor functions remain active to ensure the user is aware of their surroundings.
- Once the response is received, it is converted to speech and played through the speaker, temporarily pausing XM125 and HCSR04 alerts to avoid overlapping sounds.

### Additional Features:

1. **Web Interfaces**:
   - A webpage displaying real-time data from the four sensors (accelerometer, gyroscope, XM125, and HCSR04) for monitoring sensor status.
   - Another webpage stores and allows playback of audio files recorded after a fall incident.


# 智能拐杖项目

本项目为智能拐杖，为了帮助视障人群可以安心地行走在这个世界上。拐杖集成了树莓派、麦克风、扬声器，以及三个传感器：HCSR04超声波测距传感器、XM125脉冲相干雷达毫米波传感器（用于测量障碍物距离）、MPU6050六轴加速度计和陀螺仪（也称为IMU）。

### 功能与特点：

#### 1. **HCSR04超声波传感器**：
- 用于检测台阶的变化，防止用户被绊倒或踩空。
- 当传感器检测到距离减小时，扬声器会提示“Upstairs”。
- 当传感器检测到距离增大时，扬声器会提示“Downstairs”。

#### 2. **XM125雷达传感器**：
- 检测前方区域的障碍物并返回距离数据。
- 当障碍物进入检测范围时，扬声器发出单频声音。
- 随着距离的接近，单频声音的频率逐渐提高，为用户提供精确的距离反馈。
- 采用单频声音是因为视障人群对这种声音的敏感度较高，同时在嘈杂的环境中也更容易区分障碍物的距离。

#### 3. **MPU6050 IMU**：
- 检测拐杖的运动状态，判断用户是否处于行走或静止状态。
- 由于超声波传感器的物理局限性，仅在用户拐杖静止时进行台阶检测，以避免频繁误报。
- 检测到摔倒时触发以下两项操作：
  1. 通过Ntfy向用户家人发送通知（纯文本消息）。
  2. 将10秒的环境录音上传至GCP，可通过网站访问。
- 录音期间，暂停XM125和HCSR04的声音提醒，确保录音质量，有助于家人了解用户的环境状况。

#### 4. **语音助手**：
- 通过轻轻旋转拐杖激活语音助手。
- 用户可以直接对拐杖讲话，与语音助手交互。
- 与大语言模型对话期间，暂停XM125和HCSR04的声音提醒，确保麦克风录音清晰。
- 用户的语音会被转换为文本与大语言模型交互。在等待回复时，传感器功能保持正常，以确保用户随时感知周围环境。
- 收到回复后，拐杖将文字转换为语音通过扬声器播放，此时会暂停XM125和HCSR04的声音提醒，避免声音混淆。

### 附加功能：

1. **网页界面**：
   - 一个网页显示四个传感器的实时数据（加速度、陀螺仪、XM125、HCSR04），方便实时监控。
   - 另一个网页存储用户摔倒后上传的音频文件，可点击播放。


How to use:
RPi:
1. source ~/my_env/bin/activate
2. python sensor.py
3. python process_data.py


GCP:
gunicorn -w 4 -b 0.0.0.0:5000 app:app


Website:
Sensor:
http://34.72.243.54:5000

Fall audios:
http://34.72.243.54:5000/fall_audios

