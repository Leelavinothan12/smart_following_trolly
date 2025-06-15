# 🛒 Smart Follow-Me Shopping Cart

An autonomous AI-powered robotic trolley system designed to assist users by intelligently following them throughout a shopping session, using real-time computer vision and embedded control systems.

---

## 🚀 Project Overview

This project aims to improve the shopping experience—particularly for elderly individuals and those with mobility challenges—by developing a smart cart that automatically follows a user wearing a unique visual marker. The system integrates computer vision, real-time object tracking, obstacle avoidance, and autonomous mobility on a Raspberry Pi-powered robotic base.

---

## 🎯 Key Features

- 🎯 Marker-based person following using YOLOv5 Nano + OpenCV
- 🧠 Edge AI processing using TensorFlow Lite on Raspberry Pi
- 📷 Real-time object detection via Raspberry Pi Camera
- 🛑 Ultrasonic obstacle avoidance system (HC-SR04)
- 🔌 Web-based live feed and diagnostics with Flask
- ⚙️ Smooth navigation using PID-like motor control
- 🔋 Long-lasting power via 12V Li-ion battery
- 💻 Fully autonomous operation, no remote required

---

## 🧠 Tech Stack

| Component          | Technology |
|-------------------|------------|
| Computer Vision    | YOLOv5 Nano (TFLite) + OpenCV |
| Controller         | Raspberry Pi 4 Model B (8GB) |
| Sensors            | HC-SR04 Ultrasonic Sensor |
| Motor Driver       | L298N Dual H-Bridge |
| Web Interface      | Flask |
| Programming        | Python 3 |
| Camera             | Raspberry Pi Camera Module v2 |
| Interface          | PiCamera2, RPi.GPIO |
| Streaming          | Flask Video Feed via OpenCV |

---

## 🖥️ System Architecture

1. **Vision Input**: Pi Camera detects a custom visual marker (on user's belt).
2. **Detection**: YOLOv5 Nano (converted to TensorFlow Lite) locates marker.
3. **Obstacle Detection**: Ultrasonic sensors detect obstacles in real-time.
4. **Control Logic**: Raspberry Pi sends commands to motors using GPIO.
5. **Streaming**: Flask provides a live video feed with bounding boxes and telemetry.
6. **Motion Execution**: Motors adjust direction based on marker’s position and obstacles.

---

## 📦 Hardware Requirements

| Component | Qty | Description |
|----------|-----|-------------|
| Raspberry Pi 4B (8GB) | 1 | Main processor |
| L298N Motor Driver | 1 | Dual-channel motor driver |
| DC Gear Motors | 2 | 300 RPM for movement |
| HC-SR04 | 1 | Obstacle detection |
| Pi Camera Module v2 | 1 | Vision input |
| Li-ion Power Bank (12V) | 1 | Power supply |
| Custom Belt w/ Image Marker | 1 | Worn by the user |
| Wheels + Chassis | - | Mobility and mounting |

---

## 🧰 Software Setup

### 1. Clone the Repository

```bash
git clone https://github.com/Leelavinothan12/smart_following_trolly.git
