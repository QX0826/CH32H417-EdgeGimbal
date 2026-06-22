# CH32H417-EdgeGimbal

🚀 **Distributed Heterogeneous Edge AI Gimbal Stabilization System Based on CH32H417 and TFLM**

<p align="center">
  <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/MCU-CH32H417QEU6-orange.svg" alt="MCU">
  <img src="https://img.shields.io/badge/AI--Framework-TFLM--TensorFlow-green.svg" alt="Framework">
  <img src="https://img.shields.io/badge/RealTime-30FPS-brightgreen.svg" alt="FrameRate">
</p>

`CH32H417-EdgeGimbal` is a **hierarchical heterogeneous Edge AI gimbal stabilization system** designed for IoT, robotics, and smart terminals. The system runs fully offline with key advantages including low latency, high security, and low power consumption, enabling multi-modal collaborative control through face tracking, gesture interaction, and offline voice commands.

---

## 📖 Overview

This project employs a collaborative computing architecture: "**Edge Vision Gateway + Offline Voice Processor + On-device Intelligent Decision Center**." The high-performance compute node (PC or edge companion) runs neural-network-based face tracking and gesture recognition algorithms, the ASRPRO chip processes offline voice commands, and the CH32H417 (400MHz RISC-V MCU) serves as the core center for multi-modal data fusion and precision action execution.

---

## ⚡ Features

* **🧠 On-device TFLM Inference**: Deploys a lightweight decision/classification model based on the TensorFlow Lite for Microcontrollers (TFLM) framework on the MCU for local gesture and state classification.
* **⏳ Time-Budget Management**: Introduces a device-side time-budget management algorithm to isolate real-time control tasks from AI inference tasks, guaranteeing the determinism of the servo cycle.
* **📐 Hierarchical Heterogeneous Collaboration**: Decouples compute-heavy vision models and microsecond-level hard real-time motion algorithms to run on different computing platforms, maximizing system efficiency.
* **📈 Motion State Estimation & Adaptive Frequency**: The MCU runs motion state estimation, dynamically reducing the visual gateway's inference frequency when static, saving up to 66% of edge vision computing power.
* **🌀 Dual-stage EMA Filtering & Dead-zone**: Employs input/output dual-stage Exponential Moving Average (EMA) filtering combined with dual-axis hysteresis dead-zones to eliminate visual pixel-level jitter.
* **⚡ Three-segment Velocity Convergence**: Dynamically switches control between binary search catch-up, three-segment deceleration, and signum function micro-adjustments according to angle errors, avoiding physical overshoot.
* **🔄 Dual-core Migration Ready**: Features a decoupled global data sharing structure `gimbal_shared`, perfectly suited for CH32H417 dual-core heterogeneous task migration.

---

## 🛠️ Tech Stack

| Domain | Technology / Framework / Hardware | Description |
|:---|:---|:---|
| **MCU** | WCH CH32H417QEU6 | 400MHz high-performance RISC-V core for on-device computing |
| **MCU Library** | Standard Peripheral Library | WCH official library driving hardware TIM, USART, DMA |
| **On-device AI** | TensorFlow Lite Micro (TFLM) | Performs local gesture classification and state decision inference |
| **Vision AI** | InsightFace & MediaPipe | Implements high-precision face tracking and gesture skeleton regression |
| **Vision Gateway**| Python 3.9 + OpenCV + PySerial | Handles video capturing, first-stage smoothing, and serial communication |
| **Voice Processing**| ASRPRO-01 | Offline voice chip with neural processor decoding voice commands |

---

## 📂 Project Structure

```text
├── Common/              # Dual-core shared drivers, registers, and SPL
│   └── Common/          # Shared memory communication interface (shared_gimbal)
├── Python/              # Edge vision gateway application
│   ├── config.py        # Limiting, dead-zone, and filter parameters
│   └── 识别.py          # Main entry for vision analysis and data transmitting
├── V3F/                 # MCU V3F core project (real-time control)
│   └── User/            # Servo driver and 3-segment velocity convergence (servo.c)
├── V5F/                 # MCU V5F core project (decision making)
│   ├── App/             # TFLM inference and decision layer
│   └── User/            # Multi-modal state machine interaction logic (main.c)
├── CH32H417QEU.wvsln    # MounRiver Studio solution file
├── merge_firmware.bat   # Dual-core Hex firmware merge script (Windows)
├── merge_firmware.sh    # Dual-core Hex firmware merge script (Linux)
└── 语音控制.hd          # Offline voice command configuration file
```

---

## 🔌 Hardware Connections

> [!IMPORTANT]
> Servos draw high peak currents. They must be powered by an independent external 5V-6V power supply, and the external ground must be connected to the MCU board's **GND** to share a common reference ground.

| Interface | MCU Pin | Connected Device | Purpose | Default Baud Rate |
|:---|:---|:---|:---|:---:|
| **USART8** | PE7 (RX) / PE8 (TX) | PC USB-to-UART | Main communication and handshake channel | 115200 bps |
| **USART3** | PB11 (RX) / PB10 (TX)| PC USB-to-UART | High-frequency face tracking coordinate channel | 115200 bps |
| **USART4** | PC7 (RX) / PC6 (TX) | PC USB-to-UART | Gesture recognition command channel | 9600 bps |
| **USART2** | PD6 (RX) / PD5 (TX) | Voice Module TX1/RX1 | Voice control command receiving channel | 9600 bps |
| **USART1** | PA10 (RX) / PA9 (TX) | Voice Module TX2/RX2 | Voice broadcast feedback channel | 19200 bps |
| **TIM2_CH1**| PA0 | Pitch Servo | Outputs 50Hz PWM control signals | — |
| **TIM2_CH2**| PA1 | Yaw Servo | Outputs 50Hz PWM control signals | — |

---

## 🚀 Getting Started

### 1. Compile & Flash MCU Firmware
1. Open `CH32H417QEU.wvsln` with **MounRiver Studio**.
2. Build both `V3F` and `V5F` projects to generate Hex files.
3. Double-click `merge_firmware.bat` to merge the dual-core firmwares and flash it using WCH-Link.

### 2. Run Vision Program
Install Python dependencies:
```bash
pip install opencv-python numpy pyserial mediapipe insightface onnxruntime
```
Modify the serial port config at the bottom of `Python/识别.py`, then run:
```bash
python Python/识别.py
```

---

## 🔒 Security & Privacy

The hardware and software configurations in this project serve as architectural demonstrations. Production deployment model keys, device credentials, and communication tokens should be injected via environment variables or private configurations, and must not be hard-coded into the repository. Use simulated or anonymized data for debugging.

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
