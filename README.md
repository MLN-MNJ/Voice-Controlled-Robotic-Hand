# 📘 Voice-Controlled Robotic Fingerspelling System (ASL)

A real-time Human-Robot Interaction (HRI) system that translates spoken
English into American Sign Language (ASL) fingerspelling on a 24-DoF
dexterous robotic hand.

This project integrates **Voice AI**, **Computer Vision**, and **ROS 2**
to create a seamless pipeline from *speech → text → gesture*, featuring
a custom kinematic retargeting tool and a cross-platform hardware
bridge.

------------------------------------------------------------------------

## 🚀 Features

-   **🗣️ Voice-to-Gesture Pipeline:** Real-time speech recognition
    converts spoken English into robotic hand poses (A--Z).
-   **📷 Kinematic Retargeting Tool:** Python tool (`pose_recorder.py`)
    uses **MediaPipe** to map human hand landmarks to robot-normalized
    joint angles.
-   **🌉 Cross-Platform Bridge:** Bypasses WSL 2 hardware isolation by
    streaming audio between Windows (Host) and Ubuntu (WSL 2) via a
    lightweight **Flask TCP/IP server**.
-   **🖐️ Fine-Tuned Dexterity:** Extends the
    `dexhand_gesture_controller` with 26+ calibrated ASL poses,
    including advanced thumb opposition & finger abduction.

------------------------------------------------------------------------

## 🛠️ Tech Stack

-   **Robotics Middleware:** ROS 2 Humble\
-   **Simulation:** Gazebo, RViz\
-   **Computer Vision:** MediaPipe, OpenCV\
-   **Audio Processing:** SpeechRecognition, PyAudio\
-   **Networking:** Flask, Python Requests\
-   **Languages:** Python 3, C++

------------------------------------------------------------------------

## 🧩 System Architecture

### **1. Windows (Host Machine)**

Runs a Python server (`windows_voice_server.py`) that: - Captures
microphone audio\
- Performs Speech-to-Text (STT)\
- Exposes recognized text via a REST API

### **2. WSL 2 (Ubuntu Subsystem)**

ROS 2 node (`wsl_voice_bridge.py`) polls the Windows server and
publishes recognized text as robot commands.

### **3. ROS 2 Control Layer**

The modified `gesture_controller` receives commands (e.g., `"pose_a"`)
and drives hand joint trajectories.

------------------------------------------------------------------------

## 📦 Installation & Setup

### **1. Windows Side -- Audio Server**

Install:

``` powershell
pip install SpeechRecognition pyaudio flask
```

Run:

``` powershell
python scripts/windows_voice_server.py
```

------------------------------------------------------------------------

### **2. WSL 2 -- Robot Control**

``` bash
pip install opencv-python mediapipe rclpy
```

------------------------------------------------------------------------

### **3. Clone Repository**

``` bash
git clone https://github.com/YOUR_USERNAME/Voice-Controlled-Robotic-Hand.git
cd Voice-Controlled-Robotic-Hand
```

------------------------------------------------------------------------

## 🏃‍♂️ Usage Guide

### **Step 1 --- Windows Audio Server**

``` powershell
python scripts/windows_voice_server.py
```

------------------------------------------------------------------------

### **Step 2 --- Launch Robot Simulation**

``` bash
ros2 launch dexhand_gesture_controller simulation.launch.py
```

------------------------------------------------------------------------

### **Step 3 --- Start WSL Bridge**

``` bash
python3 scripts/wsl_voice_bridge.py
```

------------------------------------------------------------------------

### **Step 4 --- Speak! 🎙️**

Robot will fingerspell your words automatically.

------------------------------------------------------------------------

## 🔧 Calibration Tool --- Pose Recorder

``` bash
python3 scripts/pose_recorder.py
```

Use webcam → make sign → press corresponding letter → auto-generated ROS
pose code.

------------------------------------------------------------------------

## 📂 Project Structure

    Voice-Controlled-Robotic-Hand/
    ├── scripts/
    │   ├── pose_recorder.py
    │   ├── wsl_voice_bridge.py
    │   └── windows_voice_server.py
    ├── ros_modifications/
    │   └── gesture_controller.py
    └── README.md

------------------------------------------------------------------------

## 🎓 Acknowledgments

Developed by **Milan Manoj**,\
Master's in Robotics, **University of Pennsylvania**.

Base robot model by **DexHand** project.
